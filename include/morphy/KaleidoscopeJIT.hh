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

#ifndef LLVM_EXECUTIONENGINE_ORC_KALEIDOSCOPEJIT_H
#define LLVM_EXECUTIONENGINE_ORC_KALEIDOSCOPEJIT_H

#include "llvm/ADT/StringRef.h"
#include "llvm/ExecutionEngine/JITSymbol.h"
#include "llvm/ExecutionEngine/Orc/CompileOnDemandLayer.h"
#include "llvm/ExecutionEngine/Orc/CompileUtils.h"
#include "llvm/ExecutionEngine/Orc/Core.h"
#include "llvm/ExecutionEngine/Orc/ExecutionUtils.h"
#include "llvm/ExecutionEngine/Orc/IRCompileLayer.h"
#include "llvm/ExecutionEngine/Orc/IRTransformLayer.h"
#include "llvm/ExecutionEngine/Orc/JITTargetMachineBuilder.h"
#include "llvm/ExecutionEngine/Orc/RTDyldObjectLinkingLayer.h"
#include "llvm/ExecutionEngine/Orc/TPCIndirectionUtils.h"
#include "llvm/ExecutionEngine/Orc/TargetProcessControl.h"
#include "llvm/ExecutionEngine/SectionMemoryManager.h"
#include "llvm/IR/DataLayout.h"
#include "llvm/IR/LLVMContext.h"
#include "llvm/IR/LegacyPassManager.h"
#include "llvm/Transforms/InstCombine/InstCombine.h"
#include "llvm/Transforms/Scalar.h"
#include "llvm/Transforms/Scalar/GVN.h"
#include <memory>

namespace morphy {

class KaleidoscopeJIT {
private:
  std::unique_ptr<llvm::orc::TargetProcessControl> TPC;
  std::unique_ptr<llvm::orc::ExecutionSession> ES;
  std::unique_ptr<llvm::orc::TPCIndirectionUtils> TPCIU;

  llvm::DataLayout DL;
  llvm::orc::MangleAndInterner Mangle;

  llvm::orc::RTDyldObjectLinkingLayer ObjectLayer;
  llvm::orc::IRCompileLayer CompileLayer;
  llvm::orc::IRTransformLayer OptimizeLayer;
  llvm::orc::CompileOnDemandLayer CODLayer;

  llvm::orc::JITDylib &MainJD;

  static void handleLazyCallThroughError();

public:
  KaleidoscopeJIT(std::unique_ptr<llvm::orc::TargetProcessControl> TPC,
                  std::unique_ptr<llvm::orc::ExecutionSession> ES,
                  std::unique_ptr<llvm::orc::TPCIndirectionUtils> TPCIU,
                  llvm::orc::JITTargetMachineBuilder JTMB, llvm::DataLayout DL);

  ~KaleidoscopeJIT();

  static llvm::Expected<std::unique_ptr<KaleidoscopeJIT>> Create();

  const llvm::DataLayout &getDataLayout() const;

  llvm::orc::JITDylib &getMainJITDylib();

  llvm::Error addModule(llvm::orc::ThreadSafeModule TSM, llvm::orc::ResourceTrackerSP RT = nullptr);
  
  llvm::Expected<llvm::JITEvaluatedSymbol> lookup(llvm::StringRef Name);

  void triggerRecompilation();

private:
  static llvm::Expected<llvm::orc::ThreadSafeModule>
  optimizeModule(llvm::orc::ThreadSafeModule TSM, const llvm::orc::MaterializationResponsibility &R);
};

} // end namespace morphy

#endif // LLVM_EXECUTIONENGINE_ORC_KALEIDOSCOPEJIT_H