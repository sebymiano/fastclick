//===- KaleidoscopeJIT.h - A simple JIT for Kaleidoscope --------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// Contains a simple JIT definition for use in the kaleidoscope tutorials.
//
//===----------------------------------------------------------------------===//

#include "morphy/KaleidoscopeJIT.hh"

#include <click/config.h>
#include <click/glue.hh>

using namespace morphy;
using namespace llvm;
using namespace llvm::orc;

KaleidoscopeJIT::KaleidoscopeJIT(std::unique_ptr<TargetProcessControl> TPC,
                  std::unique_ptr<ExecutionSession> ES,
                  std::unique_ptr<TPCIndirectionUtils> TPCIU,
                  JITTargetMachineBuilder JTMB, DataLayout DL)
      : TPC(std::move(TPC)), ES(std::move(ES)), TPCIU(std::move(TPCIU)),
        DL(std::move(DL)), Mangle(*this->ES, this->DL),
        ObjectLayer(*this->ES,
                    []() { return std::make_unique<SectionMemoryManager>(); }),
        CompileLayer(*this->ES, ObjectLayer,
                     std::make_unique<ConcurrentIRCompiler>(std::move(JTMB))),
        OptimizeLayer(*this->ES, CompileLayer, optimizeModule),
        CODLayer(*this->ES, OptimizeLayer,
                 this->TPCIU->getLazyCallThroughManager(),
                 [this] { return this->TPCIU->createIndirectStubsManager(); }),
        MainJD(this->ES->createBareJITDylib("<main>")) {
    MainJD.addGenerator(
        cantFail(DynamicLibrarySearchGenerator::GetForCurrentProcess(
            DL.getGlobalPrefix())));
  }

KaleidoscopeJIT::~KaleidoscopeJIT() {
    if (auto Err = ES->endSession())
      ES->reportError(std::move(Err));
    if (auto Err = TPCIU->cleanup())
      ES->reportError(std::move(Err));
  }

void KaleidoscopeJIT::handleLazyCallThroughError() {
    errs() << "LazyCallThrough error: Could not find function body";
    exit(1);
}

Expected<std::unique_ptr<KaleidoscopeJIT>> KaleidoscopeJIT::Create() {
    auto SSP = std::make_shared<SymbolStringPool>();
    auto TPC = SelfTargetProcessControl::Create(SSP);
    if (!TPC)
      return TPC.takeError();

    auto ES = std::make_unique<ExecutionSession>(std::move(SSP));

    auto TPCIU = TPCIndirectionUtils::Create(**TPC);
    if (!TPCIU)
      return TPCIU.takeError();

    (*TPCIU)->createLazyCallThroughManager(
        *ES, pointerToJITTargetAddress(&handleLazyCallThroughError));

    if (auto Err = setUpInProcessLCTMReentryViaTPCIU(**TPCIU))
      return std::move(Err);

    JITTargetMachineBuilder JTMB((*TPC)->getTargetTriple());

    auto DL = JTMB.getDefaultDataLayoutForTarget();
    if (!DL)
      return DL.takeError();

    return std::make_unique<KaleidoscopeJIT>(std::move(*TPC), std::move(ES),
                                             std::move(*TPCIU), std::move(JTMB),
                                             std::move(*DL));
  }

const DataLayout &KaleidoscopeJIT::getDataLayout() const { return DL; }

JITDylib &KaleidoscopeJIT::getMainJITDylib() { return MainJD; }

Error KaleidoscopeJIT::addModule(ThreadSafeModule TSM, ResourceTrackerSP RT) {
    if (!RT)
      RT = MainJD.getDefaultResourceTracker();

    return OptimizeLayer.add(RT, std::move(TSM));
}

Expected<JITEvaluatedSymbol> KaleidoscopeJIT::lookup(llvm::StringRef Name) {
    //ES->dump(llvm::errs());
    //auto mangledName = Mangle(Name.str());
    //click_chatter("Mangled name: %s", (*mangledName).str().c_str());
    //return ES->lookup({&MainJD}, Mangle(Name.str()));
    return ES->lookup({&MainJD}, Name);
}

void KaleidoscopeJIT::triggerRecompilation() {

}

Expected<ThreadSafeModule> KaleidoscopeJIT::optimizeModule(ThreadSafeModule TSM, const MaterializationResponsibility &R) {
    TSM.withModuleDo([](Module &M) {
      // Create a function pass manager.
      auto FPM = std::make_unique<legacy::FunctionPassManager>(&M);

      click_chatter("Optimizing the given module");
      // Add some optimizations.
      FPM->add(createInstructionCombiningPass());
      FPM->add(createReassociatePass());
      FPM->add(createGVNPass());
      FPM->add(createCFGSimplificationPass());
      FPM->doInitialization();

      // Run the optimizations over all functions in the module being added to
      // the JIT.
      for (auto &F : M)
        FPM->run(F);
    });

    return std::move(TSM);
  }