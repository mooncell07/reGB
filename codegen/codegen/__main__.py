import json
import re
from dataclasses import asdict, dataclass
from enum import Enum, IntEnum, auto
from io import StringIO
from string import Template

SEMICOLON = ";"
EOL = "\n"
TAB = "  "


class OperandType(IntEnum):
    REG8 = auto()
    REG16 = auto()
    REG16_INDIRECT = auto()
    IMM16_INDIRECT = auto()
    IMM8 = auto()
    IMM16 = auto()
    SIGNED_IMM8 = auto()
    BIT = auto()
    HEX = auto()
    CC = auto()


class OperandPosition(IntEnum):
    LEFT = auto()
    RIGHT = auto()


class AddressingMode(IntEnum):
    NONE = auto()  # INVALID
    IMPLIED = auto()  # INSTR
    REGISTER = auto()  # INSTR R
    REGISTER_REGISTER = auto()  # INSTR R,R
    REGISTER_IMMEDIATE = auto()  # INSTR R,N
    REGISTERPAIR = auto()  # INSTR RR
    REGISTERPAIR_REGISTERPAIR = auto()  # INSTR RR,RR
    REGISTERPAIR_IMMEDIATE16 = auto()  # INSTR RR,NN
    REGISTER_DEREF_REGISTERPAIR = auto()  # INSTR R,(RR)
    DEREF_REGISTERPAIR = auto()  # INSTR (RR)
    DEREF_REGISTERPAIR_REGISTER = auto()  # INSTR (RR),R
    U3_REGISTER = auto()  # INSTR U3,R
    U3_DEREF_REGISTERPAIR = auto()  # INSTR U3,(RR)
    DEREF_REGISTERPAIR_IMMEDIATE = auto()  # INSTR (RR),N
    DEREF_IMMEDIATE16_REGISTER = auto()  # INSTR (U16),R
    DEREF_IMMEDIATE16_REGISTERPAIR = auto()  # INSTR (U16), RR
    REGISTER_DEREF_IMMEDIATE16 = auto()  # INSTR R,(U16)


class InstructionType(IntEnum):
    IMPLIED = 0
    UNARY = 1
    BINARY = 2


class InstructionGroup(Enum):
    UNUSED = "UNUSED"
    X8_LSM = "X8/LSM"
    X16_LSM = "X16/LSM"
    X8_ALU = "X8/ALU"
    X16_ALU = "X16/ALU"
    X8_RSB = "X8/RSB"
    CONTROL_BR = "CONTROL/BR"
    CONTROL_MISC = "CONTROL/MISC"


class FlagArg(Enum):
    C = "ftC"
    Z = "ftZ"
    H = "ftH"
    N = "ftN"


OPERAND_MAP = {
    ("A", "B", "C", "D", "E", "H", "L"): OperandType.REG8,
    ("AF", "BC", "DE", "HL", "SP"): OperandType.REG16,
    ("(HL)", "(BC)", "(DE)"): OperandType.REG16_INDIRECT,
    ("(U16)",): OperandType.IMM16_INDIRECT,
    ("U16",): OperandType.IMM16,
    ("U8",): OperandType.IMM8,
    (
        "I8",
        "SP+I8",
    ): OperandType.SIGNED_IMM8,
    ("Z", "NZ", "C", "NC"): OperandType.CC,
}

ALU_OP_MAPPING = {
    "ADD": "+",
    "ADC": "+",
    "SUB": "-",
    "SBC": "-",
    "AND": "&",
    "OR": "|",
    "XOR": "^",
    "CP": "-",
    "INC": "+",
    "DEC": "-",
    "LD": "+",  # LD treated as an ADD in one of the instrs
}


@dataclass
class Flags:
    # Z and N flags are handled within the flag getter itself
    _ALU_FLAG_MAPPING = {
        "ADD": {
            "h": "(dst & 0xF) + (src & 0xF) >= 0x10",
            "c": "dst > res",
        },
        "SUB": {
            "h": "(dst & 0xF) < (src & 0xF)",
            "c": "src > dst",
        },
        "ADC": {
            "h": "((dst & 0xF) + (src & 0xF) + c) > 0xF",
            "c": "((uint16_t)dst + (uint16_t)src + (uint16_t)c) > 0xFF",
        },
        "SBC": {
            "h": "((src & 0xF) + c) > (dst & 0xF)",
            "c": "((uint16_t)src + c) > dst",
        },
        "ADD16": {
            "h": "(dst & 0xFFF) + (src & 0xFFF) > 0xFFF",
            "c": "((uint32_t)dst + (uint32_t)src) > 0xFFFF",
        },
        "ADDi8": {
            "h": "(dst & 0xF) + (src & 0xF) >= 0x10",
            "c": "(dst & 0xFF) + (src & 0xFF) > 0xFF",
        },
    }

    z_flag: str | None = None
    n_flag: str | None = None
    h_flag: str | None = None
    c_flag: str | None = None

    @classmethod
    def _build_x8_ALU_flags(cls, mnemonic: str):
        # We group the flags in two categories based on the nature of their operation.
        flag_group = "SUB" if mnemonic in ("CP", "DEC", "SBC", "SUB") else "ADD"

        # All ALU Ops use same Z Flag operation.
        z = "!res"

        # N Flag only goes up for SUB group of operations.
        n = "true" if flag_group == "SUB" else "false"

        # H is down on (OR, XOR), up on AND and an operation in others.
        h = "true" if mnemonic == "AND" else "false"

        # Get H Flag operation for ADD, SUB, ADC, SBC
        # INC, DEC, CP share same operation so we send a group instead.
        # This saves us from explictly putting their entries in the flag mapping.
        try:
            h = cls._ALU_FLAG_MAPPING[
                mnemonic if mnemonic not in ("INC", "DEC", "CP") else flag_group
            ]["h"]
        except KeyError:
            pass

        # C Flag is down in the case of AND, OR and XOR, None in case of INC, DEC and an operation in others.
        c = "false" if mnemonic in ("AND", "OR", "XOR") else None

        # Get the C Flag operation for ADD, SUB, ADC, SBC
        # Same thing happens here except INC, DEC are left unaffected.
        try:
            c = cls._ALU_FLAG_MAPPING[mnemonic if mnemonic != "CP" else "SUB"]["c"]
        except KeyError:
            pass

        return cls(z, n, h, c)

    @classmethod
    def _build_x16_ALU_flags(cls, mnemonic: str, signed: bool = False):
        if mnemonic in ("INC", "DEC"):
            return Flags()

        z = "false" if signed else None
        n = "false"
        # LD has same C and H Flag Operations as ADD
        mnemonic = "ADD" if mnemonic == "LD" else mnemonic
        # Postfixes to distinguish between x8 and x16 flags
        mnemonic += "i8" if signed else "16"
        h = cls._ALU_FLAG_MAPPING[mnemonic]["h"]
        c = cls._ALU_FLAG_MAPPING[mnemonic]["c"]

        return Flags(z, n, h, c)

    @classmethod
    def build_ALU_flags_for(cls, instr, signed: bool = False):
        if instr.group == InstructionGroup.X8_ALU:
            return cls._build_x8_ALU_flags(instr.mnemonic)
        return cls._build_x16_ALU_flags(instr.mnemonic, signed)

    @classmethod
    def build_RSB_flags_for(
        cls, mnemonic: str, dst_arg: str = "dst", res_arg: str = "res"
    ):
        # RES and SET dont affect any flag.
        if mnemonic in ("RES", "SET"):
            return Flags()

        z = f"!{res_arg}"

        # These operations unset the Z Flag.
        if mnemonic in ("RLCA", "RRCA", "RLA", "RRA"):
            z = "false"

        # N Flag is always down on every RSB instr.
        n = "false"

        # H Flag is always down except in the case of BIT
        h = "true" if mnemonic == "BIT" else "false"

        c = None

        if mnemonic in ("RL", "RLC", "SLA", "RLA", "RLCA"):
            c = f"{dst_arg} >> 7"
        elif mnemonic in ("RR", "RRC", "SRL", "SRA", "RRA", "RRCA"):
            c = f"{dst_arg} & 1"
        elif mnemonic == "SWAP":
            c = "false"

        return cls(z, n, h, c)


class ExternalFunction:
    """
    Function Mapping for functions in C code.
    Mainly consists of Bus functions, Flags, Stack and few macros.
    """

    def __init__(self, instance: str) -> None:
        self.instance = instance

    def gen_call_getFlag(self, flag: FlagArg) -> str:
        return "getFlag({})".format(flag.value)

    def gen_call_setFlag(self, flag: Flags):
        setters = StringIO()
        if flag.z_flag:
            setters.write("Zf({});".format(flag.z_flag))
        if flag.n_flag:
            setters.write("Nf({});".format(flag.n_flag))
        if flag.h_flag:
            setters.write("Hf({});".format(flag.h_flag))
        if flag.c_flag:
            setters.write("Cf({});".format(flag.c_flag))
        return setters.getvalue()

    def gen_call_read8(self, arg: str) -> str:
        return "read8({}->bus, {}, true, true)".format(self.instance, arg)

    def gen_call_write8(self, address_arg: str, value_arg: str) -> str:
        return "write8({}->bus, {}, {}, true)".format(
            self.instance, address_arg, value_arg
        )

    def gen_call_write16(self, address_arg: str, value_arg: str) -> str:
        return "write16({}->bus, {}, {})".format(self.instance, address_arg, value_arg)

    def gen_call_next_imm8(self) -> str:
        return "next8({})".format(self.instance)

    def gen_call_next_imm16(self) -> str:
        return "next16({})".format(self.instance)

    def gen_call_internal(self) -> str:
        return "internal({}->bus)".format(self.instance)

    def gen_call_push(self, value_arg: str) -> str:
        return "push({}, {})".format(self.instance, value_arg)

    def gen_call_pop(self) -> str:
        return "pop({})".format(self.instance)

    # MACROS

    def gen_call_rotateLeftBits(self, dst_arg, value_arg) -> str:
        return f"rotateLeftBits({dst_arg}, {value_arg})"

    def gen_call_rotateRightBits(self, dst_arg, value_arg) -> str:
        return f"rotateRightBits({dst_arg}, {value_arg})"

    def gen_call_BT(self, dst_arg, value_arg) -> str:
        return f"BT({dst_arg}, {value_arg})"

    def gen_call_clearBit(self, dst_arg, value_arg) -> str:
        return f"clearBit({dst_arg}, {value_arg})"

    def gen_call_setBit(self, dst_arg, value_arg) -> str:
        return f"setBit({dst_arg}, {value_arg})"


class Operand:
    def __init__(
        self, fmt: str, pos: OperandPosition, extern: ExternalFunction
    ) -> None:
        self.fmt = fmt
        self.pos = pos
        self.extern = extern

    @property
    def type(self):
        if self.fmt.isdigit():
            return OperandType.BIT
        if re.match(r"\d\d[H]", self.fmt):
            return OperandType.HEX

        for operands, t in OPERAND_MAP.items():
            if self.fmt in operands:
                return t
        else:
            raise ValueError(f"Unknown Operand: {self.fmt}")

    @property
    def qualified_format(self):
        return (
            self.fmt.replace("(", "deref_")
            .replace(")", "")
            .replace("HL+", "HLi")
            .replace("HL-", "HLd")
            .replace("+", "_")
            .replace("-", "_")
        )

    @property
    def cc(self) -> str:
        inversed = None
        if self.fmt in ("NC", "NZ"):
            inversed = True
        elif self.fmt in ("C", "Z"):
            inversed = False
        else:
            raise ValueError("bad flag: " + self.fmt)
        flag = FlagArg.C if self.fmt in ("C", "NC") else FlagArg.Z
        return ("!" if inversed else "") + self.extern.gen_call_getFlag(flag)

    def get_expr(self, write=False) -> str:
        match self.type:
            case OperandType.REG8 | OperandType.BIT | OperandType.CC:
                return self.fmt
            case OperandType.REG16:
                return self.fmt + "p" if self.fmt != "SP" else self.fmt
            case OperandType.IMM8:
                return self.extern.gen_call_next_imm8()
            case OperandType.IMM16:
                return self.extern.gen_call_next_imm16()
            case OperandType.IMM16_INDIRECT:
                return self.extern.gen_call_read8(self.extern.gen_call_next_imm16())
            case OperandType.SIGNED_IMM8:
                return "(uint16_t)(int8_t){}".format(self.extern.gen_call_next_imm8())
            case OperandType.HEX:
                return self.fmt.replace("H", "")
            case OperandType.REG16_INDIRECT:
                rp = self.fmt.replace("(", "").replace(")", "p")
                return self.extern.gen_call_read8(rp) if not write else rp

    def __repr__(self) -> str:
        return "Operand(fmt={} type={} pos={})".format(
            self.fmt, self.type.name, self.pos.name
        )


class Instruction:
    def __init__(self, entry, extern) -> None:
        self.entry = entry
        self.extern = extern

    @property
    def mnemonic(self) -> str:
        return self.entry.get("Name").split(" ")[0].upper()

    def _get_operands(self) -> tuple[Operand | None, Operand | None]:
        name = self.entry.get("Name").split(" ")

        if len(name) == 1:
            return (None, None)

        args = [i.upper() for i in name[1].split(",")]

        if len(args) == 1:
            return (
                Operand(args[0], pos=OperandPosition.LEFT, extern=self.extern),
                None,
            )

        return (
            Operand(args[0], pos=OperandPosition.LEFT, extern=self.extern),
            Operand(args[1], pos=OperandPosition.RIGHT, extern=self.extern),
        )

    @property
    def type(self) -> InstructionType:
        operands = self._get_operands()
        operand_real = list(filter(lambda op: isinstance(op, Operand), operands))
        return InstructionType(len(operand_real))

    @property
    def group(self) -> InstructionGroup:
        return InstructionGroup(self.entry.get("Group").upper())

    @property
    def lop(self) -> Operand:
        operand = self._get_operands()[0]
        assert isinstance(
            operand, Operand
        ), "lop not available for given instr type {}".format(self.mnemonic)
        return operand

    @property
    def rop(self) -> Operand:
        operand = self._get_operands()[1]
        assert isinstance(
            operand, Operand
        ), "rop not available for given instr type {}".format(self.mnemonic)
        return operand

    @property
    def qualified_name(self):
        name = self.mnemonic
        if self.type >= InstructionType.UNARY:
            name += "_" + self.lop.qualified_format

        if self.type == InstructionType.BINARY:
            name += "_" + self.rop.qualified_format

        return name


@dataclass
class FunctionSignature:
    return_type: str
    disciminator: str
    struct_name: str
    pointer_name: str


class Function:
    def __init__(self, signature: FunctionSignature, name: str):
        self.name = name
        self.signature = signature
        self.codelines = StringIO()

    def add_codeline(self, line, terminate=True):
        self.codelines.write(TAB + line + (SEMICOLON + EOL if terminate else EOL))

    def render(self):
        signature = (
            "{return_type} {disciminator}{name}({struct_name} *{pointer_name})".format(
                **asdict(self.signature), name=self.name
            )
        )
        codelines = self.codelines.getvalue()

        self.functionbuf = StringIO()
        self.functionbuf.writelines([signature, "{", EOL, codelines, "}"])
        repr = self.functionbuf.getvalue()
        self.functionbuf.close()
        return repr


class FunctionGenerator:
    def __init__(
        self, instr: Instruction, signature: FunctionSignature, extern: ExternalFunction
    ):
        self.instr = instr
        self.signature = signature
        self.extern = extern
        self.function = Function(signature=signature, name=instr.qualified_name)

    def _writeback(self, dst: Operand, code: str):
        if dst.type in (OperandType.REG16, OperandType.REG8):
            return "{} = {}".format(dst.get_expr(), code)
        return self.extern.gen_call_write8(dst.get_expr(write=True), code)

    def _get_implied_alu(self):
        flag = Flags(z_flag=None, n_flag="false", h_flag="false", c_flag=None)
        match self.instr.mnemonic:
            case "SCF":
                flag.c_flag = "true"
            case "CCF":
                flag.c_flag = "~" + self.extern.gen_call_getFlag(FlagArg.C)
            case "CPL":
                flag.n_flag = "true"
                flag.h_flag = "true"
                flag.c_flag = None
                self.function.add_codeline(f"A = ~A")
            case "DAA":
                flag = Flags(z_flag="a == 0", h_flag="false", n_flag=None, c_flag=None)
                self.function.add_codeline("uint8_t a = A")
                self.function.add_codeline(
                    "if (!{0}) {{".format(self.extern.gen_call_getFlag(FlagArg.N)),
                    terminate=False,
                )
                self.function.add_codeline(
                    "if ({0} | ((A & 0x0F) > 9)) {{ a += 6; }}".format(
                        self.extern.gen_call_getFlag(FlagArg.H)
                    ),
                    terminate=False,
                )
                self.function.add_codeline(
                    "if ({0} | (A > 0x99)) {{ a += 0x60; {1};}}".format(
                        self.extern.gen_call_getFlag(FlagArg.C),
                        self.extern.gen_call_setFlag(Flags(c_flag="true")),
                    ),
                    terminate=False,
                )
                self.function.add_codeline("} else {", terminate=False)
                self.function.add_codeline(
                    "if ({0}) {{ a -= 0x60; }}".format(
                        self.extern.gen_call_getFlag(FlagArg.C)
                    ),
                    terminate=False,
                )
                self.function.add_codeline(
                    "if ({0}) {{ a -= 6; }}".format(
                        self.extern.gen_call_getFlag(FlagArg.H)
                    ),
                    terminate=False,
                )
                self.function.add_codeline("}", terminate=False)
                self.function.add_codeline(self.extern.gen_call_setFlag(flag))
                self.function.add_codeline("A = a")
            case _:
                raise ValueError("Invalid instr: {}".format(self.instr))

        if self.instr.mnemonic != "DAA":
            code = self.extern.gen_call_setFlag(flag)
            self.function.add_codeline(code, terminate=False)
        return self.function.render()

    def _get_unary_alu(self):
        size = "uint16_t" if self.instr.group == InstructionGroup.X16_ALU else "uint8_t"
        self.function.add_codeline(
            "{} dst = {}, src = 1".format(size, self.instr.lop.get_expr())
        )
        self.function.add_codeline(
            "{} res = dst {} src".format(size, ALU_OP_MAPPING[self.instr.mnemonic])
        )
        self.function.add_codeline(self._writeback(self.instr.lop, "res"))

        flag = Flags.build_ALU_flags_for(self.instr)
        self.function.add_codeline(self.extern.gen_call_setFlag(flag), terminate=False)

        if self.instr.group == InstructionGroup.X16_ALU:
            self.function.add_codeline(self.extern.gen_call_internal())

        return self.function.render()

    def _get_binary_alu(self):
        lhs_expr = self.instr.lop.get_expr()
        rhs_expr = self.instr.rop.get_expr()
        op = ALU_OP_MAPPING[self.instr.mnemonic]

        expr = f"dst {op}"
        if self.instr.mnemonic in ("ADC", "SBC"):
            self.function.add_codeline(
                "uint8_t c = {}".format(self.extern.gen_call_getFlag(FlagArg.C))
            )
            expr += f" c {op}"
        expr += f" src"

        size = "uint16_t" if self.instr.group == InstructionGroup.X16_ALU else "uint8_t"
        self.function.add_codeline(f"{size} dst = {lhs_expr}, src = {rhs_expr}")
        self.function.add_codeline(f"{size} res = {expr}")

        if self.instr.mnemonic != "CP":
            self.function.add_codeline(f"{lhs_expr} = res")
        if self.instr.group == InstructionGroup.X16_ALU:
            self.function.add_codeline(self.extern.gen_call_internal())

        flag = Flags.build_ALU_flags_for(self.instr)
        self.function.add_codeline(self.extern.gen_call_setFlag(flag), terminate=False)

        return self.function.render()

    def _get_prefixed_rsb(self, dst_arg="dst"):
        match self.instr.mnemonic:
            case "RLC":
                return (
                    self.extern.gen_call_rotateLeftBits(dst_arg, 1)
                    + " | "
                    + "{} >> 7".format(self.extern.gen_call_getFlag(FlagArg.C))
                )
            case "RRC":
                return (
                    self.extern.gen_call_rotateRightBits(dst_arg, 1)
                    + " | "
                    + "(0x80 & {})".format(self.extern.gen_call_getFlag(FlagArg.C))
                )
            case "RL":
                return "({} << 1) | {}".format(
                    dst_arg, self.extern.gen_call_getFlag(FlagArg.C)
                )
            case "RR":
                return "({} >> 1) | ({} << 7)".format(
                    dst_arg, self.extern.gen_call_getFlag(FlagArg.C)
                )
            case "SLA":
                return "({} << 1)".format(dst_arg)
            case "SRA":
                return "({0} >> 1) | ({0} & 0x80)".format(dst_arg)
            case "SWAP":
                return self.extern.gen_call_rotateLeftBits(dst_arg, 4)
            case "SRL":
                return "{} >> 1".format(dst_arg)
            case "BIT":
                return self.extern.gen_call_BT(dst_arg, self.instr.lop.get_expr())
            case "RES":
                return self.extern.gen_call_clearBit(dst_arg, self.instr.lop.get_expr())
            case "SET":
                return self.extern.gen_call_setBit(dst_arg, self.instr.lop.get_expr())
            case "RLCA":
                return (
                    self.extern.gen_call_rotateLeftBits(dst_arg, 1)
                    + " | "
                    + self.extern.gen_call_getFlag(FlagArg.C)
                    + " >> 7"
                )
            case "RRCA":
                return (
                    self.extern.gen_call_rotateRightBits(dst_arg, 1)
                    + " | "
                    + "(0x80 & {})".format(self.extern.gen_call_getFlag(FlagArg.C))
                )
            case "RLA":
                return (
                    f"({dst_arg} << 1)"
                    + " | "
                    + self.extern.gen_call_getFlag(FlagArg.C)
                )
            case "RRA":
                return (
                    f"({dst_arg} >> 1)"
                    + " | "
                    + self.extern.gen_call_getFlag(FlagArg.C)
                    + " << 7"
                )
            case _:
                raise ValueError("Unknown RSB instr: {}".format(self.instr.mnemonic))

    def _get_RST(self):
        self.function.add_codeline(self.extern.gen_call_internal())
        self.function.add_codeline(
            "uint16_t address = 0x{}".format(self.instr.lop.get_expr())
        )
        self.function.add_codeline(self.extern.gen_call_push("PC"))
        self.function.add_codeline("PC = address")
        return self.function.render()

    def _get_RETI(self):
        self.function.add_codeline(
            "{}->IMERising = true".format(self.signature.pointer_name)
        )
        self.function.add_codeline("opRET({})".format(self.signature.pointer_name))
        return self.function.render()

    def _get_RET_CC(self):
        self.function.add_codeline(
            "if ({}) {{".format(self.instr.lop.cc), terminate=False
        )
        self.function.add_codeline("PC = {}".format(self.extern.gen_call_pop()))
        self.function.add_codeline(self.extern.gen_call_internal())
        self.function.add_codeline("}", terminate=False)
        self.function.add_codeline(self.extern.gen_call_internal())
        return self.function.render()

    def _get_RET(self):
        if self.instr.type == InstructionType.UNARY:
            return self._get_RET_CC()
        self.function.add_codeline("PC = {}".format(self.extern.gen_call_pop()))
        self.function.add_codeline(self.extern.gen_call_internal())
        return self.function.render()

    def _get_JP_NN(self):
        self.function.add_codeline(
            "uint16_t address = {}".format(self.extern.gen_call_next_imm16())
        )
        self.function.add_codeline("PC = address")
        self.function.add_codeline(self.extern.gen_call_internal())
        return self.function.render()

    def _get_JP_HL(self):
        self.function.add_codeline("PC = HLp")
        return self.function.render()

    def _get_JP_CC_NN(self):
        self.function.add_codeline(
            "uint16_t address = {}".format(self.extern.gen_call_next_imm16())
        )
        self.function.add_codeline(
            "if ({}) {{".format(self.instr.lop.cc), terminate=False
        )
        self.function.add_codeline("PC = address")
        self.function.add_codeline(self.extern.gen_call_internal())
        self.function.add_codeline("}", terminate=False)
        return self.function.render()

    def _get_JP(self):
        if self.instr.type == InstructionType.UNARY:
            if self.instr.lop.type == OperandType.REG16:
                return self._get_JP_HL()
            return self._get_JP_NN()
        return self._get_JP_CC_NN()

    def _get_CALL_NN(self):
        self.function.add_codeline(self.extern.gen_call_internal())
        self.function.add_codeline(
            "uint16_t address = {}".format(self.extern.gen_call_next_imm16())
        )
        self.function.add_codeline(self.extern.gen_call_push("PC"))
        self.function.add_codeline("PC = address")
        return self.function.render()

    def _get_CALL_CC_NN(self):
        self.function.add_codeline(
            "uint16_t address = {}".format(self.extern.gen_call_next_imm16())
        )
        self.function.add_codeline(
            "if ({}) {{".format(self.instr.lop.cc), terminate=False
        )
        self.function.add_codeline(self.extern.gen_call_push("PC"))
        self.function.add_codeline(self.extern.gen_call_internal())
        self.function.add_codeline("PC = address")
        self.function.add_codeline("}", terminate=False)
        return self.function.render()

    def _get_CALL(self):
        if self.instr.type == InstructionType.UNARY:
            return self._get_CALL_NN()
        return self._get_CALL_CC_NN()

    def _get_JR_I8(self):
        self.function.add_codeline(
            "uint16_t address = {}".format(self.instr.lop.get_expr())
        )
        self.function.add_codeline("PC += address")
        self.function.add_codeline(self.extern.gen_call_internal())
        return self.function.render()

    def _get_JR_CC_I8(self):
        self.function.add_codeline(
            "uint16_t address = {}".format(self.instr.rop.get_expr())
        )
        self.function.add_codeline(
            "if ({}) {{".format(self.instr.lop.cc), terminate=False
        )
        self.function.add_codeline("PC += address")
        self.function.add_codeline(self.extern.gen_call_internal())
        self.function.add_codeline("}", terminate=False)
        return self.function.render()

    def _get_JR(self):
        if self.instr.type == InstructionType.UNARY:
            return self._get_JR_I8()
        return self._get_JR_CC_I8()

    def _get_XX_SP_I8(self, reg: str, extra_internal: bool):
        self.function.add_codeline(
            "uint16_t dst = SP, src = {}".format(self.instr.rop.get_expr())
        )
        self.function.add_codeline("uint16_t res = src + dst")
        self.function.add_codeline(f"{reg} = res")
        self.function.add_codeline(self.extern.gen_call_internal())
        flag = Flags.build_ALU_flags_for(self.instr, signed=True)
        self.function.add_codeline(self.extern.gen_call_setFlag(flag), terminate=False)
        if extra_internal:
            self.function.add_codeline(self.extern.gen_call_internal())
        return self.function.render()

    def gen_CONTROL_MISC(self):
        match self.instr.mnemonic:
            case "NOP":
                self.function.add_codeline("return")
            case "EI":
                self.function.add_codeline(
                    "{}->IMERising = true".format(self.signature.pointer_name)
                )
            case "DI":
                self.function.add_codeline(
                    "{}->IME = false".format(self.signature.pointer_name)
                )
            case "STOP" | "HALT":
                self.function.add_codeline(
                    "{}->halted = true".format(self.signature.pointer_name)
                )
            case "PREFIX":
                ...  # handled in main
            case _:
                raise ValueError(
                    "Invalid instr passed to control misc: {}".format(
                        self.instr.mnemonic
                    )
                )
        return self.function.render()

    def gen_CONTROL_BR(self):
        match self.instr.mnemonic:
            case "RET":
                return self._get_RET()
            case "RETI":
                return self._get_RETI()
            case "RST":
                return self._get_RST()
            case "JP":
                return self._get_JP()
            case "CALL":
                return self._get_CALL()
            case "JR":
                return self._get_JR()
            case _:
                raise ValueError("Invalid instr: {}".format(self.instr))

    def gen_LSM(self):
        name = self.instr.qualified_name
        preformed = None

        # These are special cases which cant be handled by the generic handler.
        match name[3:]:
            case "deref_HLi_A":
                preformed = self.extern.gen_call_write8("(HLp++)", "A")
            case "deref_HLd_A":
                preformed = self.extern.gen_call_write8("(HLp--)", "A")
            case "A_deref_HLi":
                preformed = "A = {}".format(self.extern.gen_call_read8("(HLp++)"))
            case "A_deref_HLd":
                preformed = "A = {}".format(self.extern.gen_call_read8("(HLp--)"))
            case "deref_FF00_U8_A":
                preformed = self.extern.gen_call_write8(
                    "(0xFF00 + {})".format(self.extern.gen_call_next_imm8()), "A"
                )
            case "A_deref_FF00_U8":
                preformed = "A = {}".format(
                    self.extern.gen_call_read8(
                        "(0xFF00 + {})".format(self.extern.gen_call_next_imm8())
                    )
                )
            case "deref_FF00_C_A":
                preformed = self.extern.gen_call_write8("(0xFF00 + C)", "A")
            case "A_deref_FF00_C":
                preformed = "A = {}".format(self.extern.gen_call_read8("(0xFF00 + C)"))
            case "A_deref_U16":
                preformed = "A = {}".format(
                    self.extern.gen_call_read8(self.extern.gen_call_next_imm16())
                )
            case "deref_U16_SP":
                preformed = self.extern.gen_call_write16(
                    self.extern.gen_call_next_imm16(), "SP"
                )
            case "deref_U16_A":
                preformed = self.extern.gen_call_write8(
                    self.extern.gen_call_next_imm16(), "A"
                )
            case _:
                ...

        if preformed:
            self.function.add_codeline(preformed)
            return self.function.render()

        if name == "LD_SP_HL":
            self.function.add_codeline(self.extern.gen_call_internal())
        self.function.add_codeline(
            self._writeback(self.instr.lop, self.instr.rop.get_expr())
        )
        return self.function.render()

    def gen_ALU(self):
        match self.instr.type:
            case InstructionType.IMPLIED:
                return self._get_implied_alu()
            case InstructionType.UNARY:
                return self._get_unary_alu()
            case InstructionType.BINARY:
                return self._get_binary_alu()

    def gen_RSB(self):
        dst = (
            Operand("A", pos=OperandPosition.LEFT, extern=self.extern)
            if self.instr.mnemonic in ("RLCA", "RRCA", "RRA", "RLA")
            else (
                self.instr.lop
                if self.instr.mnemonic not in ("BIT", "RES", "SET")
                else self.instr.rop
            )
        )
        op = self._get_prefixed_rsb()

        self.function.add_codeline("uint8_t dst = {}".format(dst.get_expr()))
        self.function.add_codeline(f"uint8_t res = {op}")
        if self.instr.mnemonic != "BIT":
            self.function.add_codeline(self._writeback(dst, "res"))
        flags = Flags.build_RSB_flags_for(self.instr.mnemonic)
        self.function.add_codeline(self.extern.gen_call_setFlag(flags), terminate=False)
        return self.function.render()

    def gen_POP(self):
        self.function.add_codeline(
            "uint16_t res = {}".format(self.extern.gen_call_pop())
        )
        if (lop := self.instr.lop).get_expr() != "AFp":
            self.function.add_codeline(self._writeback(lop, "res"))
        else:
            self.function.add_codeline("A = MSB(res)")
            self.function.add_codeline("F = res & 0xFFF0")
        return self.function.render()

    def gen_PUSH(self):
        self.function.add_codeline(self.extern.gen_call_internal())
        self.function.add_codeline(self.extern.gen_call_push(self.instr.lop.get_expr()))
        return self.function.render()

    def gen_ADD_SP_I8(self):
        return self._get_XX_SP_I8("SP", extra_internal=True)

    def gen_LD_HLp_SP_I8(self):
        return self._get_XX_SP_I8("HLp", extra_internal=False)


def add_comment(entry, opcode):
    return "// {} - {} ({}) < T({}) | Tbr({}) >".format(
        hex(opcode),
        entry.get("Name"),
        entry.get("Group"),
        entry.get("TCyclesNoBranch"),
        entry.get("TCyclesBranch"),
    )


def get_slots(
    prefixed, unprefixed, signature: FunctionSignature, extern: ExternalFunction
):
    code = StringIO()
    unpref_buffer = StringIO()

    for i, entry in enumerate(unprefixed):
        instr = Instruction(entry, extern)
        if instr.mnemonic not in ("UNUSED", "PREFIX"):
            unpref_buffer.write(
                f"LUT[{hex(i)}] = "
                + signature.disciminator
                + instr.qualified_name
                + SEMICOLON
                + EOL
            )
        fngen = FunctionGenerator(instr, signature, extern)
        fngen.function.add_codeline(add_comment(entry, i), terminate=False)
        match instr.group:
            case InstructionGroup.X8_LSM | InstructionGroup.X16_LSM:
                if instr.mnemonic == "POP":
                    code.write(fngen.gen_POP())
                elif instr.mnemonic == "PUSH":
                    code.write(fngen.gen_PUSH())
                else:
                    code.write(fngen.gen_LSM())
            case InstructionGroup.X8_ALU | InstructionGroup.X16_ALU:
                if instr.qualified_name == "ADD_SP_I8":
                    code.write(fngen.gen_ADD_SP_I8())
                elif instr.qualified_name == "LD_HL_SP_I8":
                    code.write(fngen.gen_LD_HLp_SP_I8())
                else:
                    code.write(fngen.gen_ALU())
            case InstructionGroup.X8_RSB:
                code.write(fngen.gen_RSB())
            case InstructionGroup.CONTROL_MISC:
                code.write(fngen.gen_CONTROL_MISC())
            case InstructionGroup.CONTROL_BR:
                code.write(fngen.gen_CONTROL_BR())

        code.write(EOL)

    pref_buffer = StringIO()

    for i, entry in enumerate(prefixed):
        instr = Instruction(entry, extern)
        pref_buffer.write(
            f"LUT[{hex(i)}] = "
            + signature.disciminator
            + instr.qualified_name
            + SEMICOLON
            + EOL
        )
        fngen = FunctionGenerator(instr, signature, extern)
        fngen.function.add_codeline(add_comment(entry, i), terminate=False)
        match instr.group:
            case InstructionGroup.X8_RSB:
                code.write(fngen.gen_RSB())

        code.write(EOL)

    return {
        "SLOT_HANDLERS": code.getvalue(),
        "SLOT_PREFIXED": pref_buffer.getvalue(),
        "SLOT_UNPREFIXED": unpref_buffer.getvalue(),
    }


def main():
    with open("codegen/dmgops.json", "r") as tables:
        tables = json.loads(tables.read())
        unprefixed = tables["Unprefixed"]
        cbprefixed = tables["CBPrefixed"]

        signature = FunctionSignature(
            return_type="void",
            disciminator="op",
            struct_name="State",
            pointer_name="self",
        )
        extern = ExternalFunction(signature.pointer_name)
        slots = get_slots(cbprefixed, unprefixed, signature=signature, extern=extern)

    with open("codegen/template", "r") as template:
        content = template.read()
        template = Template(content)
        res = template.substitute(slots)
        print(res)


if __name__ == "__main__":
    main()
