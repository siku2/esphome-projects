import logging
from pathlib import Path

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import esp32
from esphome.const import (
    CONF_FILE,
    CONF_ID,
    CONF_MODEL,
    CONF_PATH,
    CONF_RAW_DATA_ID,
    CONF_SIZE,
    CONF_TYPE,
    CONF_URL,
)
from esphome.core import HexInt
from esphome.types import ConfigType

_LOGGER = logging.getLogger(__name__)

DOMAIN = "tflite"

CONF_ALLOCATOR = "allocator"
CONF_BUILTIN = "builtin"
CONF_INTERPRETER = "interpreter"
CONF_OP_RESOLVER = "op_resolver"

TYPE_LOCAL = "local"
TYPE_WEB = "web"

tflite_ns = cg.esphome_ns.namespace("esphome_tflite")

BuiltinOperator = tflite_ns.enum("BuiltinOperator")
MicroMutableOpResolver = tflite_ns.class_("MicroMutableOpResolver")

AllocatorComponent = tflite_ns.class_("AllocatorComponent", cg.Component)
InterpreterComponent = tflite_ns.class_("InterpreterComponent", cg.Component)
ModelComponent = tflite_ns.class_("ModelComponent", cg.Component)

BUILTIN_OPERATORS = {
    "ABS": BuiltinOperator.BuiltinOperator_ABS,
    "ADD_N": BuiltinOperator.BuiltinOperator_ADD_N,
    "ADD": BuiltinOperator.BuiltinOperator_ADD,
    "ARG_MAX": BuiltinOperator.BuiltinOperator_ARG_MAX,
    "ARG_MIN": BuiltinOperator.BuiltinOperator_ARG_MIN,
    "ASSIGN_VARIABLE": BuiltinOperator.BuiltinOperator_ASSIGN_VARIABLE,
    "ATAN2": BuiltinOperator.BuiltinOperator_ATAN2,
    "AVERAGE_POOL_2D": BuiltinOperator.BuiltinOperator_AVERAGE_POOL_2D,
    "BATCH_MATMUL": BuiltinOperator.BuiltinOperator_BATCH_MATMUL,
    "BATCH_TO_SPACE_ND": BuiltinOperator.BuiltinOperator_BATCH_TO_SPACE_ND,
    "BIDIRECTIONAL_SEQUENCE_LSTM": BuiltinOperator.BuiltinOperator_BIDIRECTIONAL_SEQUENCE_LSTM,
    "BIDIRECTIONAL_SEQUENCE_RNN": BuiltinOperator.BuiltinOperator_BIDIRECTIONAL_SEQUENCE_RNN,
    "BITCAST": BuiltinOperator.BuiltinOperator_BITCAST,
    "BITWISE_XOR": BuiltinOperator.BuiltinOperator_BITWISE_XOR,
    "BROADCAST_ARGS": BuiltinOperator.BuiltinOperator_BROADCAST_ARGS,
    "BROADCAST_TO": BuiltinOperator.BuiltinOperator_BROADCAST_TO,
    "BUCKETIZE": BuiltinOperator.BuiltinOperator_BUCKETIZE,
    "CALL_ONCE": BuiltinOperator.BuiltinOperator_CALL_ONCE,
    "CALL": BuiltinOperator.BuiltinOperator_CALL,
    "CAST": BuiltinOperator.BuiltinOperator_CAST,
    "CEIL": BuiltinOperator.BuiltinOperator_CEIL,
    "COMPLEX_ABS": BuiltinOperator.BuiltinOperator_COMPLEX_ABS,
    "CONCAT_EMBEDDINGS": BuiltinOperator.BuiltinOperator_CONCAT_EMBEDDINGS,
    "CONCATENATION": BuiltinOperator.BuiltinOperator_CONCATENATION,
    "CONV_2D": BuiltinOperator.BuiltinOperator_CONV_2D,
    "CONV_3D_TRANSPOSE": BuiltinOperator.BuiltinOperator_CONV_3D_TRANSPOSE,
    "CONV_3D": BuiltinOperator.BuiltinOperator_CONV_3D,
    "COS": BuiltinOperator.BuiltinOperator_COS,
    "CUMSUM": BuiltinOperator.BuiltinOperator_CUMSUM,
    "CUSTOM": BuiltinOperator.BuiltinOperator_CUSTOM,
    "DELEGATE": BuiltinOperator.BuiltinOperator_DELEGATE,
    "DENSIFY": BuiltinOperator.BuiltinOperator_DENSIFY,
    "DEPTH_TO_SPACE": BuiltinOperator.BuiltinOperator_DEPTH_TO_SPACE,
    "DEPTHWISE_CONV_2D": BuiltinOperator.BuiltinOperator_DEPTHWISE_CONV_2D,
    "DEQUANTIZE": BuiltinOperator.BuiltinOperator_DEQUANTIZE,
    "DILATE": BuiltinOperator.BuiltinOperator_DILATE,
    "DIV": BuiltinOperator.BuiltinOperator_DIV,
    "DYNAMIC_UPDATE_SLICE": BuiltinOperator.BuiltinOperator_DYNAMIC_UPDATE_SLICE,
    "ELU": BuiltinOperator.BuiltinOperator_ELU,
    "EMBEDDING_LOOKUP_SPARSE": BuiltinOperator.BuiltinOperator_EMBEDDING_LOOKUP_SPARSE,
    "EMBEDDING_LOOKUP": BuiltinOperator.BuiltinOperator_EMBEDDING_LOOKUP,
    "EQUAL": BuiltinOperator.BuiltinOperator_EQUAL,
    "EXP": BuiltinOperator.BuiltinOperator_EXP,
    "EXPAND_DIMS": BuiltinOperator.BuiltinOperator_EXPAND_DIMS,
    "FAKE_QUANT": BuiltinOperator.BuiltinOperator_FAKE_QUANT,
    "FILL": BuiltinOperator.BuiltinOperator_FILL,
    "FLOOR_DIV": BuiltinOperator.BuiltinOperator_FLOOR_DIV,
    "FLOOR_MOD": BuiltinOperator.BuiltinOperator_FLOOR_MOD,
    "FLOOR": BuiltinOperator.BuiltinOperator_FLOOR,
    "FULLY_CONNECTED": BuiltinOperator.BuiltinOperator_FULLY_CONNECTED,
    "GATHER_ND": BuiltinOperator.BuiltinOperator_GATHER_ND,
    "GATHER": BuiltinOperator.BuiltinOperator_GATHER,
    "GELU": BuiltinOperator.BuiltinOperator_GELU,
    "GREATER_EQUAL": BuiltinOperator.BuiltinOperator_GREATER_EQUAL,
    "GREATER": BuiltinOperator.BuiltinOperator_GREATER,
    "HARD_SWISH": BuiltinOperator.BuiltinOperator_HARD_SWISH,
    "HASHTABLE_FIND": BuiltinOperator.BuiltinOperator_HASHTABLE_FIND,
    "HASHTABLE_IMPORT": BuiltinOperator.BuiltinOperator_HASHTABLE_IMPORT,
    "HASHTABLE_LOOKUP": BuiltinOperator.BuiltinOperator_HASHTABLE_LOOKUP,
    "HASHTABLE_SIZE": BuiltinOperator.BuiltinOperator_HASHTABLE_SIZE,
    "HASHTABLE": BuiltinOperator.BuiltinOperator_HASHTABLE,
    "IF": BuiltinOperator.BuiltinOperator_IF,
    "IMAG": BuiltinOperator.BuiltinOperator_IMAG,
    "L2_NORMALIZATION": BuiltinOperator.BuiltinOperator_L2_NORMALIZATION,
    "L2_POOL_2D": BuiltinOperator.BuiltinOperator_L2_POOL_2D,
    "LEAKY_RELU": BuiltinOperator.BuiltinOperator_LEAKY_RELU,
    "LESS_EQUAL": BuiltinOperator.BuiltinOperator_LESS_EQUAL,
    "LESS": BuiltinOperator.BuiltinOperator_LESS,
    "LOCAL_RESPONSE_NORMALIZATION": BuiltinOperator.BuiltinOperator_LOCAL_RESPONSE_NORMALIZATION,
    "LOG_SOFTMAX": BuiltinOperator.BuiltinOperator_LOG_SOFTMAX,
    "LOG": BuiltinOperator.BuiltinOperator_LOG,
    "LOGICAL_AND": BuiltinOperator.BuiltinOperator_LOGICAL_AND,
    "LOGICAL_NOT": BuiltinOperator.BuiltinOperator_LOGICAL_NOT,
    "LOGICAL_OR": BuiltinOperator.BuiltinOperator_LOGICAL_OR,
    "LOGISTIC": BuiltinOperator.BuiltinOperator_LOGISTIC,
    "LSH_PROJECTION": BuiltinOperator.BuiltinOperator_LSH_PROJECTION,
    "LSTM": BuiltinOperator.BuiltinOperator_LSTM,
    "MATRIX_DIAG": BuiltinOperator.BuiltinOperator_MATRIX_DIAG,
    "MATRIX_SET_DIAG": BuiltinOperator.BuiltinOperator_MATRIX_SET_DIAG,
    "MAX_POOL_2D": BuiltinOperator.BuiltinOperator_MAX_POOL_2D,
    "MAXIMUM": BuiltinOperator.BuiltinOperator_MAXIMUM,
    "MEAN": BuiltinOperator.BuiltinOperator_MEAN,
    "MINIMUM": BuiltinOperator.BuiltinOperator_MINIMUM,
    "MIRROR_PAD": BuiltinOperator.BuiltinOperator_MIRROR_PAD,
    "MUL": BuiltinOperator.BuiltinOperator_MUL,
    "MULTINOMIAL": BuiltinOperator.BuiltinOperator_MULTINOMIAL,
    "NEG": BuiltinOperator.BuiltinOperator_NEG,
    "NON_MAX_SUPPRESSION_V4": BuiltinOperator.BuiltinOperator_NON_MAX_SUPPRESSION_V4,
    "NON_MAX_SUPPRESSION_V5": BuiltinOperator.BuiltinOperator_NON_MAX_SUPPRESSION_V5,
    "NOT_EQUAL": BuiltinOperator.BuiltinOperator_NOT_EQUAL,
    "ONE_HOT": BuiltinOperator.BuiltinOperator_ONE_HOT,
    "PACK": BuiltinOperator.BuiltinOperator_PACK,
    "PAD": BuiltinOperator.BuiltinOperator_PAD,
    "PADV2": BuiltinOperator.BuiltinOperator_PADV2,
    "PLACEHOLDER_FOR_GREATER_OP_CODES": BuiltinOperator.BuiltinOperator_PLACEHOLDER_FOR_GREATER_OP_CODES,
    "POW": BuiltinOperator.BuiltinOperator_POW,
    "PRELU": BuiltinOperator.BuiltinOperator_PRELU,
    "QUANTIZE": BuiltinOperator.BuiltinOperator_QUANTIZE,
    "RANDOM_STANDARD_NORMAL": BuiltinOperator.BuiltinOperator_RANDOM_STANDARD_NORMAL,
    "RANDOM_UNIFORM": BuiltinOperator.BuiltinOperator_RANDOM_UNIFORM,
    "RANGE": BuiltinOperator.BuiltinOperator_RANGE,
    "RANK": BuiltinOperator.BuiltinOperator_RANK,
    "READ_VARIABLE": BuiltinOperator.BuiltinOperator_READ_VARIABLE,
    "REAL": BuiltinOperator.BuiltinOperator_REAL,
    "REDUCE_ALL": BuiltinOperator.BuiltinOperator_REDUCE_ALL,
    "REDUCE_ANY": BuiltinOperator.BuiltinOperator_REDUCE_ANY,
    "REDUCE_MAX": BuiltinOperator.BuiltinOperator_REDUCE_MAX,
    "REDUCE_MIN": BuiltinOperator.BuiltinOperator_REDUCE_MIN,
    "REDUCE_PROD": BuiltinOperator.BuiltinOperator_REDUCE_PROD,
    "REDUCE_WINDOW": BuiltinOperator.BuiltinOperator_REDUCE_WINDOW,
    "RELU_0_TO_1": BuiltinOperator.BuiltinOperator_RELU_0_TO_1,
    "RELU_N1_TO_1": BuiltinOperator.BuiltinOperator_RELU_N1_TO_1,
    "RELU": BuiltinOperator.BuiltinOperator_RELU,
    "RELU6": BuiltinOperator.BuiltinOperator_RELU6,
    "RESHAPE": BuiltinOperator.BuiltinOperator_RESHAPE,
    "RESIZE_BILINEAR": BuiltinOperator.BuiltinOperator_RESIZE_BILINEAR,
    "RESIZE_NEAREST_NEIGHBOR": BuiltinOperator.BuiltinOperator_RESIZE_NEAREST_NEIGHBOR,
    "REVERSE_SEQUENCE": BuiltinOperator.BuiltinOperator_REVERSE_SEQUENCE,
    "REVERSE_V2": BuiltinOperator.BuiltinOperator_REVERSE_V2,
    "RFFT2D": BuiltinOperator.BuiltinOperator_RFFT2D,
    "RIGHT_SHIFT": BuiltinOperator.BuiltinOperator_RIGHT_SHIFT,
    "RNN": BuiltinOperator.BuiltinOperator_RNN,
    "ROUND": BuiltinOperator.BuiltinOperator_ROUND,
    "RSQRT": BuiltinOperator.BuiltinOperator_RSQRT,
    "SCATTER_ND": BuiltinOperator.BuiltinOperator_SCATTER_ND,
    "SEGMENT_SUM": BuiltinOperator.BuiltinOperator_SEGMENT_SUM,
    "SELECT_V2": BuiltinOperator.BuiltinOperator_SELECT_V2,
    "SELECT": BuiltinOperator.BuiltinOperator_SELECT,
    "SHAPE": BuiltinOperator.BuiltinOperator_SHAPE,
    "SIGN": BuiltinOperator.BuiltinOperator_SIGN,
    "SIN": BuiltinOperator.BuiltinOperator_SIN,
    "SKIP_GRAM": BuiltinOperator.BuiltinOperator_SKIP_GRAM,
    "SLICE": BuiltinOperator.BuiltinOperator_SLICE,
    "SOFTMAX": BuiltinOperator.BuiltinOperator_SOFTMAX,
    "SPACE_TO_BATCH_ND": BuiltinOperator.BuiltinOperator_SPACE_TO_BATCH_ND,
    "SPACE_TO_DEPTH": BuiltinOperator.BuiltinOperator_SPACE_TO_DEPTH,
    "SPARSE_TO_DENSE": BuiltinOperator.BuiltinOperator_SPARSE_TO_DENSE,
    "SPLIT_V": BuiltinOperator.BuiltinOperator_SPLIT_V,
    "SPLIT": BuiltinOperator.BuiltinOperator_SPLIT,
    "SQRT": BuiltinOperator.BuiltinOperator_SQRT,
    "SQUARE": BuiltinOperator.BuiltinOperator_SQUARE,
    "SQUARED_DIFFERENCE": BuiltinOperator.BuiltinOperator_SQUARED_DIFFERENCE,
    "SQUEEZE": BuiltinOperator.BuiltinOperator_SQUEEZE,
    "STABLEHLO_ABS": BuiltinOperator.BuiltinOperator_STABLEHLO_ABS,
    "STABLEHLO_ADD": BuiltinOperator.BuiltinOperator_STABLEHLO_ADD,
    "STABLEHLO_AND": BuiltinOperator.BuiltinOperator_STABLEHLO_AND,
    "STABLEHLO_BROADCAST_IN_DIM": BuiltinOperator.BuiltinOperator_STABLEHLO_BROADCAST_IN_DIM,
    "STABLEHLO_CASE": BuiltinOperator.BuiltinOperator_STABLEHLO_CASE,
    "STABLEHLO_CBRT": BuiltinOperator.BuiltinOperator_STABLEHLO_CBRT,
    "STABLEHLO_CLAMP": BuiltinOperator.BuiltinOperator_STABLEHLO_CLAMP,
    "STABLEHLO_COMPARE": BuiltinOperator.BuiltinOperator_STABLEHLO_COMPARE,
    "STABLEHLO_COMPOSITE": BuiltinOperator.BuiltinOperator_STABLEHLO_COMPOSITE,
    "STABLEHLO_CONCATENATE": BuiltinOperator.BuiltinOperator_STABLEHLO_CONCATENATE,
    "STABLEHLO_CONVERT": BuiltinOperator.BuiltinOperator_STABLEHLO_CONVERT,
    "STABLEHLO_CONVOLUTION": BuiltinOperator.BuiltinOperator_STABLEHLO_CONVOLUTION,
    "STABLEHLO_COSINE": BuiltinOperator.BuiltinOperator_STABLEHLO_COSINE,
    "STABLEHLO_CUSTOM_CALL": BuiltinOperator.BuiltinOperator_STABLEHLO_CUSTOM_CALL,
    "STABLEHLO_DIVIDE": BuiltinOperator.BuiltinOperator_STABLEHLO_DIVIDE,
    "STABLEHLO_DOT_GENERAL": BuiltinOperator.BuiltinOperator_STABLEHLO_DOT_GENERAL,
    "STABLEHLO_DYNAMIC_SLICE": BuiltinOperator.BuiltinOperator_STABLEHLO_DYNAMIC_SLICE,
    "STABLEHLO_DYNAMIC_UPDATE_SLICE": BuiltinOperator.BuiltinOperator_STABLEHLO_DYNAMIC_UPDATE_SLICE,
    "STABLEHLO_EXPONENTIAL": BuiltinOperator.BuiltinOperator_STABLEHLO_EXPONENTIAL,
    "STABLEHLO_FLOOR": BuiltinOperator.BuiltinOperator_STABLEHLO_FLOOR,
    "STABLEHLO_GATHER": BuiltinOperator.BuiltinOperator_STABLEHLO_GATHER,
    "STABLEHLO_IOTA": BuiltinOperator.BuiltinOperator_STABLEHLO_IOTA,
    "STABLEHLO_LOG": BuiltinOperator.BuiltinOperator_STABLEHLO_LOG,
    "STABLEHLO_LOGISTIC": BuiltinOperator.BuiltinOperator_STABLEHLO_LOGISTIC,
    "STABLEHLO_MAXIMUM": BuiltinOperator.BuiltinOperator_STABLEHLO_MAXIMUM,
    "STABLEHLO_MINIMUM": BuiltinOperator.BuiltinOperator_STABLEHLO_MINIMUM,
    "STABLEHLO_MULTIPLY": BuiltinOperator.BuiltinOperator_STABLEHLO_MULTIPLY,
    "STABLEHLO_NEGATE": BuiltinOperator.BuiltinOperator_STABLEHLO_NEGATE,
    "STABLEHLO_OR": BuiltinOperator.BuiltinOperator_STABLEHLO_OR,
    "STABLEHLO_PAD": BuiltinOperator.BuiltinOperator_STABLEHLO_PAD,
    "STABLEHLO_POWER": BuiltinOperator.BuiltinOperator_STABLEHLO_POWER,
    "STABLEHLO_REDUCE_WINDOW": BuiltinOperator.BuiltinOperator_STABLEHLO_REDUCE_WINDOW,
    "STABLEHLO_REDUCE": BuiltinOperator.BuiltinOperator_STABLEHLO_REDUCE,
    "STABLEHLO_REMAINDER": BuiltinOperator.BuiltinOperator_STABLEHLO_REMAINDER,
    "STABLEHLO_RESHAPE": BuiltinOperator.BuiltinOperator_STABLEHLO_RESHAPE,
    "STABLEHLO_RNG_BIT_GENERATOR": BuiltinOperator.BuiltinOperator_STABLEHLO_RNG_BIT_GENERATOR,
    "STABLEHLO_RSQRT": BuiltinOperator.BuiltinOperator_STABLEHLO_RSQRT,
    "STABLEHLO_SCATTER": BuiltinOperator.BuiltinOperator_STABLEHLO_SCATTER,
    "STABLEHLO_SELECT": BuiltinOperator.BuiltinOperator_STABLEHLO_SELECT,
    "STABLEHLO_SHIFT_LEFT": BuiltinOperator.BuiltinOperator_STABLEHLO_SHIFT_LEFT,
    "STABLEHLO_SLICE": BuiltinOperator.BuiltinOperator_STABLEHLO_SLICE,
    "STABLEHLO_SORT": BuiltinOperator.BuiltinOperator_STABLEHLO_SORT,
    "STABLEHLO_SUBTRACT": BuiltinOperator.BuiltinOperator_STABLEHLO_SUBTRACT,
    "STABLEHLO_TANH": BuiltinOperator.BuiltinOperator_STABLEHLO_TANH,
    "STABLEHLO_TRANSPOSE": BuiltinOperator.BuiltinOperator_STABLEHLO_TRANSPOSE,
    "STABLEHLO_WHILE": BuiltinOperator.BuiltinOperator_STABLEHLO_WHILE,
    "STRIDED_SLICE": BuiltinOperator.BuiltinOperator_STRIDED_SLICE,
    "SUB": BuiltinOperator.BuiltinOperator_SUB,
    "SUM": BuiltinOperator.BuiltinOperator_SUM,
    "SVDF": BuiltinOperator.BuiltinOperator_SVDF,
    "TANH": BuiltinOperator.BuiltinOperator_TANH,
    "TILE": BuiltinOperator.BuiltinOperator_TILE,
    "TOPK_V2": BuiltinOperator.BuiltinOperator_TOPK_V2,
    "TRANSPOSE_CONV": BuiltinOperator.BuiltinOperator_TRANSPOSE_CONV,
    "TRANSPOSE": BuiltinOperator.BuiltinOperator_TRANSPOSE,
    "UNIDIRECTIONAL_SEQUENCE_LSTM": BuiltinOperator.BuiltinOperator_UNIDIRECTIONAL_SEQUENCE_LSTM,
    "UNIDIRECTIONAL_SEQUENCE_RNN": BuiltinOperator.BuiltinOperator_UNIDIRECTIONAL_SEQUENCE_RNN,
    "UNIQUE": BuiltinOperator.BuiltinOperator_UNIQUE,
    "UNPACK": BuiltinOperator.BuiltinOperator_UNPACK,
    "UNSORTED_SEGMENT_MAX": BuiltinOperator.BuiltinOperator_UNSORTED_SEGMENT_MAX,
    "UNSORTED_SEGMENT_MIN": BuiltinOperator.BuiltinOperator_UNSORTED_SEGMENT_MIN,
    "UNSORTED_SEGMENT_PROD": BuiltinOperator.BuiltinOperator_UNSORTED_SEGMENT_PROD,
    "UNSORTED_SEGMENT_SUM": BuiltinOperator.BuiltinOperator_UNSORTED_SEGMENT_SUM,
    "VAR_HANDLE": BuiltinOperator.BuiltinOperator_VAR_HANDLE,
    "WHERE": BuiltinOperator.BuiltinOperator_WHERE,
    "WHILE": BuiltinOperator.BuiltinOperator_WHILE,
    "ZEROS_LIKE": BuiltinOperator.BuiltinOperator_ZEROS_LIKE,
}


def _validate_unique_operators(value: list[str]) -> list[str]:
    if len(value) != len(set(value)):
        raise cv.Invalid("Duplicate operators are not allowed")
    return value


OP_RESOLVER_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ID): cv.declare_id(MicroMutableOpResolver),
        cv.Required(CONF_BUILTIN): cv.All(
            cv.ensure_list(cv.enum(BUILTIN_OPERATORS, upper=True)),
            _validate_unique_operators,
        ),
    }
)

LOCAL_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_PATH): cv.file_,
    }
)

WEB_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_URL): cv.url,
    }
)

TYPED_FILE_SCHEMA = cv.typed_schema(
    {
        TYPE_LOCAL: LOCAL_SCHEMA,
        TYPE_WEB: WEB_SCHEMA,
    }
)


def _file_schema(value: ConfigType | str) -> ConfigType:
    if isinstance(value, str):
        return _validate_file_shorthand(value)
    return TYPED_FILE_SCHEMA(value)


def _validate_file_shorthand(value: str) -> ConfigType:
    if value.startswith("http://") or value.startswith("https://"):
        return _file_schema(
            {
                CONF_TYPE: TYPE_WEB,
                CONF_URL: value,
            }
        )
    return _file_schema(
        {
            CONF_TYPE: TYPE_LOCAL,
            CONF_PATH: value,
        }
    )


MODEL_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ID): cv.declare_id(ModelComponent),
        cv.Required(CONF_FILE): _file_schema,
        cv.GenerateID(CONF_RAW_DATA_ID): cv.declare_id(cg.uint8),
    }
)

ALLOCATOR_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ID): cv.declare_id(AllocatorComponent),
        cv.Required(CONF_SIZE): cv.All(cv.validate_bytes, cv.positive_int),
    }
)

INTERPRETER_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ID): cv.declare_id(InterpreterComponent),
        cv.GenerateID(CONF_MODEL): cv.use_id(ModelComponent),
    }
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_OP_RESOLVER): OP_RESOLVER_SCHEMA,
        cv.Required(CONF_ALLOCATOR): ALLOCATOR_SCHEMA,
        cv.Required(CONF_MODEL): cv.ensure_list(MODEL_SCHEMA),
        cv.Required(CONF_INTERPRETER): cv.ensure_list(INTERPRETER_SCHEMA),
    }
)


async def to_code(config: ConfigType) -> None:
    # Using the same versions as micro_wake_word!
    esp32.add_idf_component(name="espressif/esp-tflite-micro", ref="1.3.3~1")
    esp32.add_idf_component(name="espressif/esp-nn", ref="1.1.2")
    cg.add_build_flag("-DESP_NN")

    resolver_config: ConfigType = config[CONF_OP_RESOLVER]
    MicroMutableOpResolverT = MicroMutableOpResolver.template(
        len(resolver_config[CONF_BUILTIN])
    )
    resolver = cg.new_variable(
        resolver_config[CONF_ID],
        MicroMutableOpResolverT(),
        MicroMutableOpResolverT,
        static=True,
    )
    for op_name in resolver_config[CONF_BUILTIN]:
        op = BUILTIN_OPERATORS[op_name]
        cg.add(tflite_ns.add_builtin_operator.template(op)(resolver))

    allocator_config: ConfigType = config[CONF_ALLOCATOR]
    allocator = cg.new_Pvariable(allocator_config[CONF_ID])
    await cg.register_component(allocator, allocator_config)
    cg.add(allocator.set_size(allocator_config[CONF_SIZE]))

    for model_config in config[CONF_MODEL]:
        p: Path = model_config[CONF_FILE][CONF_PATH]
        rhs = [HexInt(x) for x in p.read_bytes()]
        buf = cg.progmem_array(model_config[CONF_RAW_DATA_ID], rhs)
        model = cg.new_Pvariable(model_config[CONF_ID])
        await cg.register_component(model, model_config)
        cg.add(model.set_buf(buf, len(rhs)))

    for interpreter_config in config[CONF_INTERPRETER]:
        model = await cg.get_variable(interpreter_config[CONF_MODEL])
        interpreter = cg.new_Pvariable(interpreter_config[CONF_ID])
        await cg.register_component(interpreter, interpreter_config)
        cg.add(interpreter.set_op_resolver(resolver))
        cg.add(interpreter.set_model_component(model))
        cg.add(interpreter.set_allocator_component(allocator))
