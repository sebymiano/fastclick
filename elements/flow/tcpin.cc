#include <click/config.h>
#include <click/router.hh>
#include <click/args.hh>
#include <click/error.hh>
#include <clicknet/tcp.h>
#include <click/timestamp.hh>
#include "tcpin.hh"
#include "ipelement.hh"

CLICK_DECLS

#define TCP_TIMEOUT 30000

TCPIn::TCPIn() : outElement(NULL), returnElement(NULL),
    poolFcbTcpCommon(),
    tableFcbTcpCommon(),
    _allow_resize(false)
{
    // Initialize the memory pools of each thread
    for(unsigned int i = 0; i < poolModificationNodes.weight(); ++i)
        poolModificationNodes.get_value(i).initialize(MODIFICATIONNODES_POOL_SIZE);

    for(unsigned int i = 0; i < poolModificationLists.weight(); ++i)
        poolModificationLists.get_value(i).initialize(MODIFICATIONLISTS_POOL_SIZE);

    // Warning about the fact that the system must be integrated to Middleclick in order to
    // work properly.
    #ifndef HAVE_FLOW
        click_chatter("WARNING: You are using a version of this program that is not compatible"
            " with the flow management system provided by Middleclick. Therefore, you will"
            " only be able to use one flow.");
    #endif
}

TCPIn::~TCPIn() {

}

int TCPIn::configure(Vector<String> &conf, ErrorHandler *errh)
{
    String returnName = "";
    String outName = "";
    int flowDirectionParam = -1;

    if(Args(conf, this, errh)
    .read_mp("FLOWDIRECTION", flowDirectionParam)
    .read_mp("OUTNAME", outName)
    .read_mp("RETURNNAME", returnName)
    .read("ALLOW_RESIZE", _allow_resize)
    .complete() < 0)
        return -1;

    Element* returnElement = this->router()->find(returnName, errh);
    Element* outElement = this->router()->find(outName, errh);

    if(returnElement == NULL)
    {
        click_chatter("Error: Could not find TCPIn element "
            "called \"%s\".", returnName.c_str());
        return -1;
    }
    else if(outElement == NULL)
    {
        click_chatter("Error: Could not find TCPOut element "
            "called \"%s\".", outName.c_str());
        return -1;
    }
    else if(strcmp("TCPIn", returnElement->class_name()) != 0)
    {
        click_chatter("Error: Element \"%s\" is not a TCPIn element "
            "but a %s element.", returnName.c_str(), returnElement->class_name());
        return -1;
    }
    else if(strcmp("TCPOut", outElement->class_name()) != 0)
    {
        click_chatter("Error: Element \"%s\" is not a TCPOut element "
            "but a %s element.", outName.c_str(), outElement->class_name());
        return -1;
    }
    else if(flowDirectionParam != 0 && flowDirectionParam != 1)
    {
        click_chatter("Error: FLOWDIRECTION %u is not valid.", flowDirectionParam);
        return -1;
    }
    else
    {
        this->returnElement = (TCPIn*)returnElement;
        this->outElement = (TCPOut*)outElement;
        this->returnElement->add_remote_element(this);
        this->outElement->add_remote_element(this);
        if (this->outElement->setInElement(this, errh) != 0)
            return -1;
        setFlowDirection((unsigned int)flowDirectionParam);
        this->outElement->setFlowDirection(getFlowDirection());
    }

    return 0;
}

void TCPIn::push_batch(int port, fcb_tcpin* fcb_in, PacketBatch* flow)
{
    // Assign the tcp_common structure if not already done
#if DEBUG_TCP
    click_chatter("Fcb in : %p, Common : %p, Batch : %p", fcb_in, fcb_in->common,flow);
#endif
    auto fnt = [this,fcb_in](Packet* p) -> Packet* {
        if(fcb_in->common == NULL)
        {
eagain:
            if(!assignTCPCommon(p))
            {
                if (isRst(p)) {
                    //click_chatter("RST received");
                    //First packet was a RST
                    outElement->output_push_batch(0, PacketBatch::make_from_packet(p)); //Elements never knew about this flow, we bypass

                    //TCPReorder will ensure the RST is alone, so we don't need to bother about dropping the rest of the batch, there is none
                } else {
                // The allocation failed, meaning that the packet is not a SYN
                // packet. This is not supposed to happen and it means that
                // the first two packets of the connection are not SYN packets
                    click_chatter("Warning: Trying to assign a common tcp memory area"
                        " for a non-SYN packet or a non-matching tupple");
                    p->kill();
                }

                return NULL;
            }

            //If not syn, drop the flow
            if(!isSyn(p)) { //TODO : move to top block?
                WritablePacket* packet = p->uniqueify();
                closeConnection(packet, false);
//                click_chatter("Packet is not syn, closing connection");
                packet->kill();
                return NULL;
            }

            // Manage the TCP options:
            // - Remove the SACK-permitted option
            // - Detect the window scale
            // - Detect MSS
            WritablePacket* packet = p->uniqueify();
            manageOptions(packet);
            p = packet;

            if (isAck(p) && fcb_in->common->state == TCPState::ESTABLISHING) {
                fcb_in->common->lastAckReceived[getFlowDirection()] = getAckNumber(p); //We need to do it now befor the GT check for the OPEN state
            }
        }
        else // At least one packet of this side of the flow has been seen
        {
            // The structure has been assigned so the three-way handshake is over
            // Check that the packet is not a SYN packet
            if(isSyn(p))
            {
                if (fcb_in->common->state == TCPState::CLOSED) {
                    click_chatter("Renewing !");
                    SFCB_STACK(
                    release_tcp_internal(fcb_save);
                    );
                    fcb_in->common = 0;
                    goto eagain;
                } else {
                    click_chatter("Warning: Unexpected SYN packet (state %d, is_ack : %d). Dropping it",fcb_in->common->state, isAck(p));
                    p->kill();
                    return NULL;
                }
            } else if (isAck(p) && fcb_in->common->state == TCPState::ESTABLISHING) {
                fcb_in->common->lastAckReceived[getFlowDirection()] = getAckNumber(p); //We need to do it now befor the GT check for the OPEN state
                fcb_in->common->state = TCPState::OPEN; //No need to lock, only us can write this
            }

        }


        if(checkConnectionClosed(p))
        {
            //click_chatter("Connection is already closed");
            p->kill();
            return NULL;
        }


        if (_allow_resize) {
            //TODO : fine-grain this lock
            fcb_in->common->lock.acquire();

            WritablePacket *packet = p->uniqueify();

            // Set the annotation indicating the initial ACK value
            setInitialAck(packet, getAckNumber(packet));

            // Compute the offset of the TCP payload
            uint16_t offset = getPayloadOffset(packet);
            packet->setContentOffset(offset);

            ByteStreamMaintainer &maintainer = fcb_in->common->maintainers[getFlowDirection()];
            ByteStreamMaintainer &otherMaintainer = fcb_in->common->maintainers[getOppositeFlowDirection()];

            // Update the window size
            uint16_t prevWindowSize = maintainer.getWindowSize();
            uint16_t newWindowSize = getWindowSize(packet);
            maintainer.setWindowSize(newWindowSize);

            tcp_seq_t seqNumber = getSequenceNumber(packet);

            if(otherMaintainer.isLastAckSentSet())
            {
                tcp_seq_t lastAckSentOtherSide = otherMaintainer.getLastAckSent();

                if(!isSyn(packet) && SEQ_LT(seqNumber, lastAckSentOtherSide))
                {
                    // We receive content that has already been ACKed.
                    // This case occurs when the ACK is lost between the middlebox and
                    // the destination.
                    // In this case, we re-ACK the content and we discard it
                    ackPacket(packet);
                    packet->kill();
                    fcb_in->common->lock.release();
                    return NULL;
                }
            }

            // Take care of the ACK value in the packet
            bool isAnAck = isAck(packet);
            tcp_seq_t ackNumber = 0;
            tcp_seq_t newAckNumber = 0;
            if(isAnAck)
            {
                // Map the ack number according to the ByteStreamMaintainer of the other direction
                ackNumber = getAckNumber(packet);
                newAckNumber = otherMaintainer.mapAck(ackNumber);

                // Check the value of the previous ack received if it exists
                bool lastAckReceivedSet = fcb_in->common->lastAckReceivedSet();
                uint32_t prevLastAckReceived = 0;
                if(lastAckReceivedSet)
                    prevLastAckReceived = fcb_in->common->getLastAckReceived(getFlowDirection());


                // Check if we acknowledged new data
                if(lastAckReceivedSet && SEQ_GT(ackNumber, prevLastAckReceived))
                {
                    // Increase congestion window
                    uint64_t cwnd = otherMaintainer.getCongestionWindowSize();
                    uint64_t ssthresh = otherMaintainer.getSsthresh();
                    // Sender segment size
                    uint16_t mss = otherMaintainer.getMSS();
                    uint64_t increase = 0;

                    // Check if we are in slow start mode
                    if(cwnd <= ssthresh)
                        increase = mss;
                    else
                        increase = mss * mss / cwnd;

                    otherMaintainer.setCongestionWindowSize(cwnd + increase);

                    maintainer.setDupAcks(0);
                }

                // Update the value of the last ACK received
                fcb_in->common->setLastAckReceived(getFlowDirection(),getAckNumber(p));

                // Prune the ByteStreamMaintainer of the other side
                otherMaintainer.prune(ackNumber);

                // Update the statistics about the RTT
                // And potentially update the retransmission timer
                fcb_in->common->lock.release();
                fcb_in->common->retransmissionTimings[getOppositeFlowDirection()].signalAck(ackNumber);
                fcb_in->common->lock.acquire();

                // Check if the current packet is just an ACK without additional information
                if(isJustAnAck(packet) && prevWindowSize == newWindowSize)
                {
                    bool isDup = false;
                    // Check duplicate acks
                    if(prevLastAckReceived == ackNumber)
                    {
                        isDup = true;
                        uint8_t dupAcks = maintainer.getDupAcks();
                        dupAcks++;
                        maintainer.setDupAcks(dupAcks);

                        // Fast retransmit
                        if(dupAcks >= 3)
                        {
                            fcb_in->common->lock.release();
                            fcb_in->common->retransmissionTimings[getOppositeFlowDirection()].fireNow();
                            fcb_in->common->lock.acquire();
                            maintainer.setDupAcks(0);
                        }
                    }

                    // Check that the ACK value is greater than what we have already
                    // sent to the destination.
                    // If this is a duplicate ACK, we don't discard it so that the mechanism
                    // of fast retransmission is preserved
                    if(maintainer.isLastAckSentSet() && SEQ_LEQ(newAckNumber, maintainer.getLastAckSent()) && !isDup)
                    {
                        // If this is not the case, the packet does not bring any additional information
                        // We can drop it
                        packet->kill();
                        fcb_in->common->lock.release();
                        return NULL;
                    }
                }

                // If needed, update the ACK value in the packet with the mapped one
                if(ackNumber != newAckNumber)
                    setAckNumber(packet, newAckNumber);
            }
            fcb_in->common->lock.release();
            return packet;
        } else { //Resize not allowed
            // Compute the offset of the TCP payload

            if (isAck(p)) {
                fcb_in->common->setLastAckReceived(getFlowDirection(),getAckNumber(p));
            }
            uint16_t offset = getPayloadOffset(p);
            p->setContentOffset(offset);

            return p;
        }
    };
    auto drop = [](Packet* p){};
    EXECUTE_FOR_EACH_PACKET_DROPPABLE(fnt, flow, drop);
    if (flow)
        output(0).push_batch(flow);
}

int TCPIn::initialize(ErrorHandler *errh) {
    if (StackSpaceElement<fcb_tcpin>::initialize(errh) != 0)
        return -1;
    if (get_passing_threads(true).weight() <= 1) {
        tableFcbTcpCommon.disable_mt();
    }
    return 0;
}

TCPOut* TCPIn::getOutElement()
{
    return outElement;
}

TCPIn* TCPIn::getReturnElement()
{
    return returnElement;
}

void TCPIn::release_tcp_internal(FlowControlBlock* fcb) {
    auto fcb_in = fcb_data_for(fcb);
    auto &common = fcb_in->common;
    if (fcb_in->conn_release_fnt) {
        fcb_in->conn_release_fnt(fcb,fcb_in->conn_release_thunk);
        fcb_in->conn_release_fnt = 0;
    }
     common->lock.acquire();
    //The last one release common
    if (--common->use_count == 0) {
        common->lock.release();
        //click_chatter("Ok, I'm releasing common ;)");
        common->~tcp_common();
        //lock->acquire();
        //tin->tableTcpCommon->erase(flowID); //TODO : consume if it was not taken by the other side
        poolFcbTcpCommon.release(common);
        fcb_in->common = 0;
        //lock->release();
    }
    else
        common->lock.release();

}

void TCPIn::release_tcp(FlowControlBlock* fcb, void* thunk) {
    TCPIn* tin = static_cast<TCPIn*>(thunk);
    auto fcb_in = tin->fcb_data_for(fcb);
    //click_chatter("Releasing TCP flow");
    //TODO : clean
    // Put back in the corresponding memory pool all the modification lists
    // in use (in the hashtable)
    /*for(HashTable<tcp_seq_t, ModificationList*>::iterator it = modificationLists.begin();
        it != modificationLists.end(); ++it)
    {
        // Call the destructor to release the object's own memory
        (it.value())->~ModificationList();
        // Put it back in the pool
        poolModificationLists->releaseMemory(it.value());
    }

*/
    tin->release_tcp_internal(fcb);

    //TODO CALL  chain
    if (fcb_in->previous_fnt)
        fcb_in->previous_fnt(fcb, fcb_in->previous_thunk);

}

/**
 * Remove timeout and release fct. Common is destroyed but
 *  the FCB stays up until all packets are freed.
 */
void TCPIn::releaseFCBState() {
    //click_chatter("TCP is closing, killing state");
    fcb_release_timeout();
    fcb_remove_release_fnt(fcb_data(), &release_tcp);
    assert(fcb_data()->common);
    SFCB_STACK(
    release_tcp_internal(fcb_save);
    );
}

void TCPIn::closeConnection(Packet *packet, bool graceful)
{
    auto fcb_in = fcb_data();
    uint8_t newFlag = 0;

    if(graceful)
        newFlag = TH_FIN;
    else
        newFlag = TH_RST;



    click_tcp tcph = *packet->tcp_header();

    // Change the flags of the packet
    tcph.th_flags = tcph.th_flags | newFlag;

    TCPState::Value newState;

    fcb_in->common->lock.acquire();
    if(!graceful)
    {
        newState = TCPState::CLOSED;
    } else {
        newState = TCPState::BEING_CLOSED_GRACEFUL_1;
    }
    fcb_in->common->state = newState;

    //Send FIN or RST to other side
        // Get the information needed to ack the given packet
        uint32_t saddr = getDestinationAddress(packet);
        uint32_t daddr = getSourceAddress(packet);
        uint16_t sport = getDestinationPort(packet);
        uint16_t dport = getSourcePort(packet);
        // The SEQ value is the initial ACK value in the packet sent
        // by the source.
        tcp_seq_t seq = getInitialAck(packet);

        // The ACK is the sequence number sent by the source
        // to which we add the size of the payload in order to acknowledge it
        tcp_seq_t ack = getSequenceNumber(packet) + getPayloadLength(packet);

        if(isFin(packet) || isSyn(packet))
            ack++;

        // Craft and send the ack
        outElement->sendClosingPacket(fcb_in->common->maintainers[getOppositeFlowDirection()],
            saddr, daddr, sport, dport, seq, ack, graceful);

    //click_chatter("Closing connection on flow %u (graceful: %u)",        getFlowDirection(), graceful);

    fcb_in->common->lock.release();

    if (!graceful) { //This is the last time this side will see a packet, release
        releaseFCBState();
    }

    StackElement::closeConnection(packet, graceful);
}

ModificationList* TCPIn::getModificationList(WritablePacket* packet)
{
    auto fcb_in = fcb_data();
    auto modificationLists = fcb_in->modificationLists;

    ModificationList* list = NULL;

    // Search the modification list in the hashtable
    HashTable<tcp_seq_t, ModificationList*>::iterator it =
        modificationLists->find(getSequenceNumber(packet));

    // If we could find the element
    if(it != modificationLists->end())
        list = it.value();

    // If no list was assigned to this packet, create a new one
    if(list == NULL)
    {
        ModificationList* listPtr = poolModificationLists->getMemory();
        // Call the constructor manually to have a clean object
        list = new(listPtr) ModificationList(&(*poolModificationNodes));
        modificationLists->set(getSequenceNumber(packet), list);
    }

    return list;
}

bool TCPIn::hasModificationList(Packet* packet)
{
    auto fcb_in = fcb_data();
    auto modificationLists = fcb_in->modificationLists;
    HashTable<tcp_seq_t, ModificationList*>::iterator it =
        modificationLists->find(getSequenceNumber(packet));

    return (it != modificationLists->end());
}

bool TCPIn::allowResize() {
    return _allow_resize;
}

void TCPIn::removeBytes(WritablePacket* packet, uint32_t position, uint32_t length)
{
    if (unlikely(!_allow_resize)) {
        click_chatter("ERROR : An element is trying to insert bytes while TCPIn was configured without allowing modifications. Insertion ignored.");
        return;
    }
    ModificationList* list = getModificationList(packet);

    tcp_seq_t seqNumber = getSequenceNumber(packet);

    // Used to have the position in the TCP flow and not in the packet
    uint16_t tcpOffset = getPayloadOffset(packet);

    uint16_t contentOffset = packet->getContentOffset();
    position += contentOffset;
    list->addModification(seqNumber, seqNumber + position - tcpOffset, -((int)length));

    unsigned char *source = packet->data();
    if(position > packet->length())
    {
        click_chatter("Error: Invalid removeBytes call (packet length: %u, position: %u)",
            packet->length(), position);
        return;
    }
    uint32_t bytesAfter = packet->length() - position;

    memmove(&source[position], &source[position + length], bytesAfter);
    packet->take(length);

    // Continue in the stack function
    StackElement::removeBytes(packet, position, length);
}

WritablePacket* TCPIn::insertBytes(WritablePacket* packet, uint32_t position,
     uint32_t length)
{
    if (unlikely(!_allow_resize)) {
        click_chatter("ERROR : An element is trying to insert bytes while TCPIn was configured without allowing modifications. Insertion ignored.");
        return packet;
    }
    tcp_seq_t seqNumber = getSequenceNumber(packet);

    uint16_t tcpOffset = getPayloadOffset(packet);
    uint16_t contentOffset = packet->getContentOffset();
    position += contentOffset;
    getModificationList(packet)->addModification(seqNumber, seqNumber + position - tcpOffset,
         (int)length);

    uint32_t bytesAfter = packet->length() - position;
    WritablePacket *newPacket = packet->put(length);
    assert(newPacket != NULL);
    unsigned char *source = newPacket->data();

    memmove(&source[position + length], &source[position], bytesAfter);

    return newPacket;
}

void TCPIn::requestMorePackets(Packet *packet, bool force)
{
    click_chatter("TCP requestMorePackets");
    ackPacket(packet, force);

    // Continue in the stack function
    StackElement::requestMorePackets(packet, force);
}

void TCPIn::ackPacket(Packet* packet, bool force)
{
    auto fcb_in = fcb_data();
    // Get the information needed to ack the given packet
    uint32_t saddr = getDestinationAddress(packet);
    uint32_t daddr = getSourceAddress(packet);
    uint16_t sport = getDestinationPort(packet);
    uint16_t dport = getSourcePort(packet);
    // The SEQ value is the initial ACK value in the packet sent
    // by the source.
    tcp_seq_t seq;
    if (_allow_resize) {
        seq = getInitialAck(packet);
    } else {
        seq = getAckNumber(packet);
    }

    // The ACK is the sequence number sent by the source
    // to which we add the size of the payload in order to acknowledge it
    tcp_seq_t ack = getSequenceNumber(packet) + getPayloadLength(packet);

    if(isFin(packet) || isSyn(packet))
        ack++;

    fcb_in->common->lock.acquire();
    // Craft and send the ack
    outElement->sendAck(fcb_in->common->maintainers[getOppositeFlowDirection()], saddr, daddr,
        sport, dport, seq, ack, force);
    fcb_in->common->lock.release();
}

bool TCPIn::checkConnectionClosed(Packet *packet)
{
    auto fcb_in = fcb_data();

    TCPState::Value state = fcb_in->common->state; //Read-only access, no need to lock

    //click_chatter("Connection state is %d", state);
    // If the connection is open, we just check if the packet is a FIN. If it is we go to the hard sequence.
    if (state == TCPState::OPEN)
    {
        if (isFin(packet) || isRst(packet)) {
            //click_chatter("Connection is closing, we received a FIN or RST in open state");
            goto do_check;
        }
        return false; //Let the FIN through
    }

    do_check:
    fcb_in->common->lock.acquire(); //Re read with lock if not in fast path
    state = fcb_in->common->state;

    if (isRst(packet)) {
        fcb_in->common->state = TCPState::CLOSED;
        fcb_in->common->lock.release();
        return false;
    }

    if(state == TCPState::OPEN) {
        //click_chatter("TCP is now closing with the first FIN");
        fcb_in->common->state = TCPState::BEING_CLOSED_GRACEFUL_1;
        fcb_in->common->lock.release();
        return false; //Let the FIN through. We cannot release now as there is an ACK that needs to come
    // If the connection is being closed and we have received the last packet, close it completely
    } else if(state == TCPState::BEING_CLOSED_GRACEFUL_1)
    {
        if(isFin(packet)) {
            //click_chatter("Connection is being closed gracefully, this is the second FIN");
            fcb_in->common->state = TCPState::BEING_CLOSED_GRACEFUL_2;
        }
        fcb_in->common->lock.release();
        return false; //Let the packet through anyway
    }
    else if(state == TCPState::BEING_CLOSED_GRACEFUL_2)
    {
        //click_chatter("Connection is being closed gracefully, this is the last ACK");
        fcb_in->common->state = TCPState::CLOSED;
        fcb_in->common->lock.release();
        return false; //We need the out element to eventually correct the ACK number
    }
/*    else if(state == TCPState::BEING_CLOSED_UNGRACEFUL)
    {
        if(isRst(packet)) { //RST to a RST, should not happen but handle anyway
            click_chatter("Connection is being closed ungracefully, due to one of our RST on the other side");
            fcb_in->common->state = TCPState::CLOSED;
            fcb_in->common->lock.release();
            return false; //Let the RST go through so next elements can fast-clean
        }
    }*/

    fcb_in->common->lock.release();
    return true;
}

unsigned int TCPIn::determineFlowDirection()
{
    return getFlowDirection();
}

bool TCPIn::registerConnectionClose(StackReleaseChain* fcb_chain, SubFlowRealeaseFnt fnt, void* thunk)
{
    auto fcb_in = fcb_data();
    fcb_chain->previous_fnt = fcb_in->conn_release_fnt;
    fcb_chain->previous_thunk = fcb_in->conn_release_thunk;
    fcb_in->conn_release_fnt = fnt;
    fcb_in->conn_release_thunk = thunk;
    return true;
}

bool TCPIn::assignTCPCommon(Packet *packet)
{
    auto fcb_in = fcb_data();

    const click_tcp *tcph = packet->tcp_header();
    uint8_t flags = tcph->th_flags;
    const click_ip *iph = packet->ip_header();

    // The data in the flow will start at current sequence number
    uint32_t flowStart = getSequenceNumber(packet);

    // Check if we are the side initiating the connection or not
    // (if ACK flag, we are not the initiator)
    if(((flags & TH_ACK && flags & TH_SYN)) || flags & TH_RST)
    {
        //click_chatter("SynAck or RST"); //For both we need the matching connection, for RST to close it if any

        // Get the flow ID for the opposite side of the connection
        IPFlowID flowID(iph->ip_dst, tcph->th_dport, iph->ip_src, tcph->th_sport);

        // Get the struct allocated by the initiator, and remove it if found
        fcb_in->common = returnElement->getTCPCommon(flowID);

        if (fcb_in->common == 0) //No matching connection
            return false;

        if (flags & TH_RST) {
            fcb_in->common->state = TCPState::CLOSED;
            fcb_in->common = 0;
            //We have no choice here but to rely on the other side timing out, this is better than traversing the tree of the other side. When waking up,
            //it will see that the state is being closed ungracefull and need to cleans
            return false; //Note that RST will be catched on return and will still go through, as the dest needs to know the flow is rst
        }

        fcb_in->common->use_count++;

        if (_allow_resize) {
            // Initialize the RBT with the RBTManager
            fcb_in->common->maintainers[getFlowDirection()].initialize(&(*rbtManager), flowStart);
        }
        //click_chatter("RE Common is %p",fcb_in->common);
    }
    else
    {
        if(!(flags & TH_SYN)) {//First packet, not rst and not syn... Discard
            //click_chatter("Not syn !");
            return false;
        }

        IPFlowID flowID(iph->ip_src, tcph->th_sport, iph->ip_dst, tcph->th_dport);
        // We are the initiator, so we need to allocate memory
        tcp_common *allocated = poolFcbTcpCommon.allocate();

        // Add an entry if the hashtable
        tableFcbTcpCommon.find_insert(flowID, allocated);

        // Set the pointer in the structure
        fcb_in->common = allocated;
        fcb_in->common->use_count++;
        if (_allow_resize) {
            // Initialize the RBT with the RBTManager
            fcb_in->common->maintainers[getFlowDirection()].initialize(&(*rbtManager), flowStart);
        }

        // Store in our structure the information needed to free the memory
        // of the common structure
        //fcb_in->flowID = flowID;

        //click_chatter("AL Common is %p",fcb_in->common);
    }

    fcb_acquire_timeout(TCP_TIMEOUT);
    fcb_set_release_fnt(fcb_in, release_tcp);

    // Set information about the flow
    fcb_in->common->maintainers[getFlowDirection()].setIpSrc(getSourceAddress(packet));
    fcb_in->common->maintainers[getFlowDirection()].setIpDst(getDestinationAddress(packet));
    fcb_in->common->maintainers[getFlowDirection()].setPortSrc(getSourcePort(packet));
    fcb_in->common->maintainers[getFlowDirection()].setPortDst(getDestinationPort(packet));

    return true;
}

bool TCPIn::isLastUsefulPacket(Packet *packet)
{
    if(isFin(packet) || isRst(packet))
        return true;
    else
        return false;
}

tcp_common* TCPIn::getTCPCommon(IPFlowID flowID)
{
    tcp_common* p;
    bool it = tableFcbTcpCommon.find_remove(flowID,p);

    if(!it)
    {
        return NULL; // Not in the table
    }
    else
    {
        return p;
    }
}

void TCPIn::manageOptions(WritablePacket *packet)
{
    auto fcb_in = fcb_data();

    click_tcp *tcph = packet->tcp_header();

    uint8_t *optStart = (uint8_t *) (tcph + 1);
    uint8_t *optEnd = (uint8_t *) tcph + (tcph->th_off << 2);

    if(optEnd > packet->end_data())
        optEnd = packet->end_data();

    while(optStart < optEnd)
    {
        if(optStart[0] == TCPOPT_EOL) // End of list
            break; // Stop searching
        else if(optStart[0] == TCPOPT_NOP)
            optStart += 1; // Move to the next option
        else if(optStart[1] < 2 || optStart[1] + optStart > optEnd)
            break; // Avoid malformed options
        else if(optStart[0] == TCPOPT_SACK_PERMITTED && optStart[1] == TCPOLEN_SACK_PERMITTED)
        {
            uint32_t old_hw = *((uint16_t*)optStart);
            // If we find the SACK permitted option, we remove it
            for(int i = 0; i < TCPOLEN_SACK_PERMITTED; ++i) {
                optStart[i] = TCPOPT_NOP; // Replace the option with NOP
            }
            uint32_t new_hw = (TCPOPT_NOP << 8) + TCPOPT_NOP;

            click_update_in_cksum(&tcph->th_sum, old_hw, new_hw);
            //click_chatter("SACK Permitted removed from options");

            optStart += optStart[1];
        }
        else if(optStart[0] == TCPOPT_WSCALE && optStart[1] == TCPOLEN_WSCALE)
        {
            if (_allow_resize) {
                uint16_t winScale = optStart[2];

                if(winScale >= 1)
                    winScale = 2 << (winScale - 1);

                fcb_in->common->maintainers[flowDirection].setWindowScale(winScale);
                fcb_in->common->maintainers[flowDirection].setUseWindowScale(true);

                //click_chatter("Window scaling set to %u for flow %u", winScale, flowDirection);

                if(isAck(packet))
                {
                    // Here, we have a SYNACK
                    // It means that we know if the other side of the flow
                    // has the option enabled
                    // if this is not the case, we disable it for this side as well
                    if(!fcb_in->common->maintainers[getOppositeFlowDirection()].getUseWindowScale())
                    {
                        fcb_in->common->maintainers[flowDirection].setUseWindowScale(false);
                        //click_chatter("Window scaling disabled");
                    }
                }
            }
            optStart += optStart[1];
        }
        else if(optStart[0] == TCPOPT_MAXSEG && optStart[1] == TCPOLEN_MAXSEG)
        {

            if (_allow_resize) {
                uint16_t mss = (optStart[2] << 8) | optStart[3];
                fcb_in->common->maintainers[flowDirection].setMSS(mss);

                //click_chatter("MSS for flow %u: %u", flowDirection, mss);
                fcb_in->common->maintainers[flowDirection].setCongestionWindowSize(mss);
            }

            optStart += optStart[1];
        }
        else
            optStart += optStart[1]; // Move to the next option
    }
}

void TCPIn::setFlowDirection(unsigned int flowDirection)
{
    this->flowDirection = flowDirection;
}


CLICK_ENDDECLS
ELEMENT_REQUIRES(TCPElement)
EXPORT_ELEMENT(TCPIn)
ELEMENT_MT_SAFE(TCPIn)