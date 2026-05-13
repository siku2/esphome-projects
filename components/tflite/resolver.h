#pragma once

#include <esphome/core/log.h>

#include "tflite.h"

namespace esphome::esphome_tflite {

// clang-format off
#define TFLITE_BUILTIN_OPS(X)                                              \
  X(ABS,                          AddAbs)                                   \
  X(ADD_N,                        AddAddN)                                  \
  X(ADD,                          AddAdd)                                   \
  X(ARG_MAX,                      AddArgMax)                                \
  X(ARG_MIN,                      AddArgMin)                                \
  X(ASSIGN_VARIABLE,              AddAssignVariable)                        \
  X(AVERAGE_POOL_2D,              AddAveragePool2D)                         \
  X(BATCH_MATMUL,                 AddBatchMatMul)                           \
  X(BATCH_TO_SPACE_ND,            AddBatchToSpaceNd)                        \
  X(BROADCAST_ARGS,               AddBroadcastArgs)                         \
  X(BROADCAST_TO,                 AddBroadcastTo)                           \
  X(CALL_ONCE,                    AddCallOnce)                              \
  X(CAST,                         AddCast)                                  \
  X(CEIL,                         AddCeil)                                  \
  X(CONCATENATION,                AddConcatenation)                         \
  X(CONV_2D,                      AddConv2D)                                \
  X(COS,                          AddCos)                                   \
  X(CUMSUM,                       AddCumSum)                                \
  X(DEPTH_TO_SPACE,               AddDepthToSpace)                          \
  X(DEPTHWISE_CONV_2D,            AddDepthwiseConv2D)                       \
  X(DEQUANTIZE,                   AddDequantize)                            \
  X(DIV,                          AddDiv)                                   \
  X(ELU,                          AddElu)                                   \
  X(EMBEDDING_LOOKUP,             AddEmbeddingLookup)                       \
  X(EQUAL,                        AddEqual)                                 \
  X(EXP,                          AddExp)                                   \
  X(EXPAND_DIMS,                  AddExpandDims)                            \
  X(FILL,                         AddFill)                                  \
  X(FLOOR_DIV,                    AddFloorDiv)                              \
  X(FLOOR_MOD,                    AddFloorMod)                              \
  X(FLOOR,                        AddFloor)                                 \
  X(FULLY_CONNECTED,              AddFullyConnected)                        \
  X(GATHER_ND,                    AddGatherNd)                              \
  X(GATHER,                       AddGather)                                \
  X(GREATER_EQUAL,                AddGreaterEqual)                          \
  X(GREATER,                      AddGreater)                               \
  X(HARD_SWISH,                   AddHardSwish)                             \
  X(IF,                           AddIf)                                    \
  X(L2_NORMALIZATION,             AddL2Normalization)                       \
  X(L2_POOL_2D,                   AddL2Pool2D)                              \
  X(LEAKY_RELU,                   AddLeakyRelu)                             \
  X(LESS_EQUAL,                   AddLessEqual)                             \
  X(LESS,                         AddLess)                                  \
  X(LOG_SOFTMAX,                  AddLogSoftmax)                            \
  X(LOG,                          AddLog)                                   \
  X(LOGICAL_AND,                  AddLogicalAnd)                            \
  X(LOGICAL_NOT,                  AddLogicalNot)                            \
  X(LOGICAL_OR,                   AddLogicalOr)                             \
  X(LOGISTIC,                     AddLogistic)                              \
  X(MAX_POOL_2D,                  AddMaxPool2D)                             \
  X(MAXIMUM,                      AddMaximum)                               \
  X(MEAN,                         AddMean)                                  \
  X(MINIMUM,                      AddMinimum)                               \
  X(MIRROR_PAD,                   AddMirrorPad)                             \
  X(MUL,                          AddMul)                                   \
  X(NEG,                          AddNeg)                                   \
  X(NOT_EQUAL,                    AddNotEqual)                              \
  X(PACK,                         AddPack)                                  \
  X(PAD,                          AddPad)                                   \
  X(PADV2,                        AddPadV2)                                 \
  X(PRELU,                        AddPrelu)                                 \
  X(QUANTIZE,                     AddQuantize)                              \
  X(READ_VARIABLE,                AddReadVariable)                          \
  X(REDUCE_MAX,                   AddReduceMax)                             \
  X(RELU,                         AddRelu)                                  \
  X(RELU6,                        AddRelu6)                                 \
  X(RESHAPE,                      AddReshape)                               \
  X(RESIZE_BILINEAR,              AddResizeBilinear)                        \
  X(RESIZE_NEAREST_NEIGHBOR,      AddResizeNearestNeighbor)                 \
  X(ROUND,                        AddRound)                                 \
  X(RSQRT,                        AddRsqrt)                                 \
  X(SELECT_V2,                    AddSelectV2)                              \
  X(SHAPE,                        AddShape)                                 \
  X(SIN,                          AddSin)                                   \
  X(SLICE,                        AddSlice)                                 \
  X(SOFTMAX,                      AddSoftmax)                               \
  X(SPACE_TO_BATCH_ND,            AddSpaceToBatchNd)                        \
  X(SPACE_TO_DEPTH,               AddSpaceToDepth)                          \
  X(SPLIT_V,                      AddSplitV)                                \
  X(SPLIT,                        AddSplit)                                 \
  X(SQRT,                         AddSqrt)                                  \
  X(SQUARE,                       AddSquare)                                \
  X(SQUARED_DIFFERENCE,           AddSquaredDifference)                     \
  X(SQUEEZE,                      AddSqueeze)                               \
  X(STRIDED_SLICE,                AddStridedSlice)                          \
  X(SUB,                          AddSub)                                   \
  X(SUM,                          AddSum)                                   \
  X(SVDF,                         AddSvdf)                                  \
  X(TANH,                         AddTanh)                                  \
  X(TRANSPOSE_CONV,               AddTransposeConv)                         \
  X(TRANSPOSE,                    AddTranspose)                             \
  X(UNIDIRECTIONAL_SEQUENCE_LSTM, AddUnidirectionalSequenceLSTM)           \
  X(UNPACK,                       AddUnpack)                                \
  X(VAR_HANDLE,                   AddVarHandle)                             \
  X(WHILE,                        AddWhile)                                 \
  X(ZEROS_LIKE,                   AddZerosLike)
// clang-format on

template<BuiltinOperator op> struct builtin_op_adder;

#define DEFINE_OP_ADDER(EnumSuffix, Method) \
  template<> struct builtin_op_adder<BuiltinOperator_##EnumSuffix> { \
    template<unsigned int N> static TfLiteStatus add(MicroMutableOpResolver<N> &r) { return r.Method(); } \
  };

TFLITE_BUILTIN_OPS(DEFINE_OP_ADDER)
#undef DEFINE_OP_ADDER

template<BuiltinOperator op, unsigned int tOpCount>
void add_builtin_operator(MicroMutableOpResolver<tOpCount> &resolver) {
  TfLiteStatus status = builtin_op_adder<op>::add(resolver);
  if (status != kTfLiteOk)
    ESP_LOGE("tflite", "Failed to add operator %d: error %d", static_cast<int>(op), static_cast<int>(status));
}

}  // namespace esphome::esphome_tflite
