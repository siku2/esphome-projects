import hashlib
import logging
from functools import partial
from pathlib import Path

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import external_files
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
from esphome.core import CORE, HexInt
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
    "AVERAGE_POOL_2D": BuiltinOperator.BuiltinOperator_AVERAGE_POOL_2D,
    "BATCH_MATMUL": BuiltinOperator.BuiltinOperator_BATCH_MATMUL,
    "BATCH_TO_SPACE_ND": BuiltinOperator.BuiltinOperator_BATCH_TO_SPACE_ND,
    "BROADCAST_ARGS": BuiltinOperator.BuiltinOperator_BROADCAST_ARGS,
    "BROADCAST_TO": BuiltinOperator.BuiltinOperator_BROADCAST_TO,
    "CALL_ONCE": BuiltinOperator.BuiltinOperator_CALL_ONCE,
    "CAST": BuiltinOperator.BuiltinOperator_CAST,
    "CEIL": BuiltinOperator.BuiltinOperator_CEIL,
    "CONCATENATION": BuiltinOperator.BuiltinOperator_CONCATENATION,
    "CONV_2D": BuiltinOperator.BuiltinOperator_CONV_2D,
    "COS": BuiltinOperator.BuiltinOperator_COS,
    "CUMSUM": BuiltinOperator.BuiltinOperator_CUMSUM,
    "DEPTH_TO_SPACE": BuiltinOperator.BuiltinOperator_DEPTH_TO_SPACE,
    "DEPTHWISE_CONV_2D": BuiltinOperator.BuiltinOperator_DEPTHWISE_CONV_2D,
    "DEQUANTIZE": BuiltinOperator.BuiltinOperator_DEQUANTIZE,
    "DIV": BuiltinOperator.BuiltinOperator_DIV,
    "ELU": BuiltinOperator.BuiltinOperator_ELU,
    "EMBEDDING_LOOKUP": BuiltinOperator.BuiltinOperator_EMBEDDING_LOOKUP,
    "EQUAL": BuiltinOperator.BuiltinOperator_EQUAL,
    "EXP": BuiltinOperator.BuiltinOperator_EXP,
    "EXPAND_DIMS": BuiltinOperator.BuiltinOperator_EXPAND_DIMS,
    "FILL": BuiltinOperator.BuiltinOperator_FILL,
    "FLOOR_DIV": BuiltinOperator.BuiltinOperator_FLOOR_DIV,
    "FLOOR_MOD": BuiltinOperator.BuiltinOperator_FLOOR_MOD,
    "FLOOR": BuiltinOperator.BuiltinOperator_FLOOR,
    "FULLY_CONNECTED": BuiltinOperator.BuiltinOperator_FULLY_CONNECTED,
    "GATHER_ND": BuiltinOperator.BuiltinOperator_GATHER_ND,
    "GATHER": BuiltinOperator.BuiltinOperator_GATHER,
    "GREATER_EQUAL": BuiltinOperator.BuiltinOperator_GREATER_EQUAL,
    "GREATER": BuiltinOperator.BuiltinOperator_GREATER,
    "HARD_SWISH": BuiltinOperator.BuiltinOperator_HARD_SWISH,
    "IF": BuiltinOperator.BuiltinOperator_IF,
    "L2_NORMALIZATION": BuiltinOperator.BuiltinOperator_L2_NORMALIZATION,
    "L2_POOL_2D": BuiltinOperator.BuiltinOperator_L2_POOL_2D,
    "LEAKY_RELU": BuiltinOperator.BuiltinOperator_LEAKY_RELU,
    "LESS_EQUAL": BuiltinOperator.BuiltinOperator_LESS_EQUAL,
    "LESS": BuiltinOperator.BuiltinOperator_LESS,
    "LOG_SOFTMAX": BuiltinOperator.BuiltinOperator_LOG_SOFTMAX,
    "LOG": BuiltinOperator.BuiltinOperator_LOG,
    "LOGICAL_AND": BuiltinOperator.BuiltinOperator_LOGICAL_AND,
    "LOGICAL_NOT": BuiltinOperator.BuiltinOperator_LOGICAL_NOT,
    "LOGICAL_OR": BuiltinOperator.BuiltinOperator_LOGICAL_OR,
    "LOGISTIC": BuiltinOperator.BuiltinOperator_LOGISTIC,
    "MAX_POOL_2D": BuiltinOperator.BuiltinOperator_MAX_POOL_2D,
    "MAXIMUM": BuiltinOperator.BuiltinOperator_MAXIMUM,
    "MEAN": BuiltinOperator.BuiltinOperator_MEAN,
    "MINIMUM": BuiltinOperator.BuiltinOperator_MINIMUM,
    "MIRROR_PAD": BuiltinOperator.BuiltinOperator_MIRROR_PAD,
    "MUL": BuiltinOperator.BuiltinOperator_MUL,
    "NEG": BuiltinOperator.BuiltinOperator_NEG,
    "NOT_EQUAL": BuiltinOperator.BuiltinOperator_NOT_EQUAL,
    "PACK": BuiltinOperator.BuiltinOperator_PACK,
    "PAD": BuiltinOperator.BuiltinOperator_PAD,
    "PADV2": BuiltinOperator.BuiltinOperator_PADV2,
    "PRELU": BuiltinOperator.BuiltinOperator_PRELU,
    "QUANTIZE": BuiltinOperator.BuiltinOperator_QUANTIZE,
    "READ_VARIABLE": BuiltinOperator.BuiltinOperator_READ_VARIABLE,
    "REDUCE_MAX": BuiltinOperator.BuiltinOperator_REDUCE_MAX,
    "RELU": BuiltinOperator.BuiltinOperator_RELU,
    "RELU6": BuiltinOperator.BuiltinOperator_RELU6,
    "RESHAPE": BuiltinOperator.BuiltinOperator_RESHAPE,
    "RESIZE_BILINEAR": BuiltinOperator.BuiltinOperator_RESIZE_BILINEAR,
    "RESIZE_NEAREST_NEIGHBOR": BuiltinOperator.BuiltinOperator_RESIZE_NEAREST_NEIGHBOR,
    "ROUND": BuiltinOperator.BuiltinOperator_ROUND,
    "RSQRT": BuiltinOperator.BuiltinOperator_RSQRT,
    "SELECT_V2": BuiltinOperator.BuiltinOperator_SELECT_V2,
    "SHAPE": BuiltinOperator.BuiltinOperator_SHAPE,
    "SIN": BuiltinOperator.BuiltinOperator_SIN,
    "SLICE": BuiltinOperator.BuiltinOperator_SLICE,
    "SOFTMAX": BuiltinOperator.BuiltinOperator_SOFTMAX,
    "SPACE_TO_BATCH_ND": BuiltinOperator.BuiltinOperator_SPACE_TO_BATCH_ND,
    "SPACE_TO_DEPTH": BuiltinOperator.BuiltinOperator_SPACE_TO_DEPTH,
    "SPLIT_V": BuiltinOperator.BuiltinOperator_SPLIT_V,
    "SPLIT": BuiltinOperator.BuiltinOperator_SPLIT,
    "SQRT": BuiltinOperator.BuiltinOperator_SQRT,
    "SQUARE": BuiltinOperator.BuiltinOperator_SQUARE,
    "SQUARED_DIFFERENCE": BuiltinOperator.BuiltinOperator_SQUARED_DIFFERENCE,
    "SQUEEZE": BuiltinOperator.BuiltinOperator_SQUEEZE,
    "STRIDED_SLICE": BuiltinOperator.BuiltinOperator_STRIDED_SLICE,
    "SUB": BuiltinOperator.BuiltinOperator_SUB,
    "SUM": BuiltinOperator.BuiltinOperator_SUM,
    "SVDF": BuiltinOperator.BuiltinOperator_SVDF,
    "TANH": BuiltinOperator.BuiltinOperator_TANH,
    "TRANSPOSE_CONV": BuiltinOperator.BuiltinOperator_TRANSPOSE_CONV,
    "TRANSPOSE": BuiltinOperator.BuiltinOperator_TRANSPOSE,
    "UNIDIRECTIONAL_SEQUENCE_LSTM": BuiltinOperator.BuiltinOperator_UNIDIRECTIONAL_SEQUENCE_LSTM,
    "UNPACK": BuiltinOperator.BuiltinOperator_UNPACK,
    "VAR_HANDLE": BuiltinOperator.BuiltinOperator_VAR_HANDLE,
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


def _compute_local_file_path(value: ConfigType) -> Path:
    url: str = value[CONF_URL]
    h = hashlib.new("sha256")
    h.update(url.encode())
    key = h.hexdigest()[:8]
    base_dir = external_files.compute_local_file_dir(DOMAIN)
    return base_dir / key


def _file_schema(value: ConfigType | str) -> ConfigType:
    if isinstance(value, str):
        return _validate_file_shorthand(value)
    return TYPED_FILE_SCHEMA(value)


def _get_file_path(config: ConfigType) -> Path:
    ty = config[CONF_TYPE]
    if ty == TYPE_WEB:
        return _compute_local_file_path(config)
    if ty == TYPE_LOCAL:
        return CORE.relative_config_path(config[CONF_PATH])
    raise cv.Invalid("Invalid file type")


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

MODEL_FILES_SCHEMA = cv.All(
    cv.ensure_list(MODEL_SCHEMA),
    partial(
        external_files.download_web_files_in_config,
        path_for=_compute_local_file_path,
    ),
)


async def _model_file_to_code(config: ConfigType) -> cg.MockObj:
    p = _get_file_path(config[CONF_FILE])
    rhs = [HexInt(x) for x in p.read_bytes()]
    buf = cg.progmem_array(config[CONF_RAW_DATA_ID], rhs)
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_buf(buf, len(rhs)))
    return var


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
        cv.Required(CONF_MODEL): MODEL_FILES_SCHEMA,
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
        await _model_file_to_code(model_config)

    for interpreter_config in config[CONF_INTERPRETER]:
        model = await cg.get_variable(interpreter_config[CONF_MODEL])
        interpreter = cg.new_Pvariable(interpreter_config[CONF_ID])
        await cg.register_component(interpreter, interpreter_config)
        cg.add(interpreter.set_op_resolver(resolver))
        cg.add(interpreter.set_model_component(model))
        cg.add(interpreter.set_allocator_component(allocator))
