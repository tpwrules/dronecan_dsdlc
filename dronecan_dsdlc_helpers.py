#!/usr/bin/env python
# encoding: utf-8

'''
Copyright Jonathan Challinger, 2021
Copyright Siddharth Bharat Purohit, CubePilot Pty. Ltd. 2021
Released under GNU GPL version 3 or later
'''
import sys
import os
import errno
import em
import math
import copy
sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.realpath(__file__)), "../pydronecan/"))
import dronecan

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

def get_empy_env_request(msg):
    assert msg.kind == msg.KIND_SERVICE
    msg_underscored_name = msg.full_name.replace('.','_')+'Request'
    msg_resp_underscored_name = msg.full_name.replace('.','_')+'Response'
    msg_define_name = msg.full_name.replace('.','_')+'_REQUEST'
    return {
        'msg': msg,
        'msg_underscored_name': msg_underscored_name,
        'msg_header_file_name': msg_header_name_request(msg),
        'msg_c_type': underscored_name_to_ctype(msg_underscored_name),
        'msg_cpp_type': underscored_name_to_cpptype(msg_underscored_name),
        'msg_union': msg.request_union,
        'msg_fields': msg.request_fields,
        'msg_constants': msg.request_constants,
        'msg_max_bitlen': msg.get_max_bitlen_request(),
        'msg_dt_sig': msg.get_data_type_signature(),
        'msg_default_dtid': msg.default_dtid,
        'msg_kind': 'request',
        'msg_resp_underscored_name': msg_resp_underscored_name,
        'msg_resp_header_file_name': msg_header_name_response(msg),
        'msg_define_name': msg_define_name
    }

def get_empy_env_response(msg):
    assert msg.kind == msg.KIND_SERVICE
    msg_underscored_name = msg.full_name.replace('.','_')+'Response'
    msg_define_name = msg.full_name.replace('.','_')+'_RESPONSE'
    return {
        'msg': msg,
        'msg_underscored_name': msg_underscored_name,
        'msg_header_file_name': msg_header_name_response(msg),
        'msg_c_type': underscored_name_to_ctype(msg_underscored_name),
        'msg_cpp_type': underscored_name_to_cpptype(msg_underscored_name),
        'msg_union': msg.response_union,
        'msg_fields': msg.response_fields,
        'msg_constants': msg.response_constants,
        'msg_max_bitlen': msg.get_max_bitlen_response(),
        'msg_dt_sig': msg.get_data_type_signature(),
        'msg_default_dtid': msg.default_dtid,
        'msg_kind': 'response',
        'msg_define_name': msg_define_name
    }

def get_empy_env_broadcast(msg):
    assert msg.kind == msg.KIND_MESSAGE
    msg_underscored_name = msg.full_name.replace('.','_')
    return {
        'msg': msg,
        'msg_underscored_name': msg_underscored_name,
        'msg_header_file_name': msg_header_name(msg),
        'msg_c_type': underscored_name_to_ctype(msg_underscored_name),
        'msg_cpp_type': underscored_name_to_cpptype(msg_underscored_name),
        'msg_union': msg.union,
        'msg_fields': msg.fields,
        'msg_constants': msg.constants,
        'msg_max_bitlen': msg.get_max_bitlen(),
        'msg_dt_sig': msg.get_data_type_signature(),
        'msg_default_dtid': msg.default_dtid,
        'msg_kind': 'broadcast',
        'msg_define_name': msg_underscored_name
    }

def dronecan_type_is_signed(dronecan_type):
    assert dronecan_type.category == dronecan_type.CATEGORY_PRIMITIVE
    if dronecan_type.kind == dronecan_type.KIND_BOOLEAN:
        return False
    elif dronecan_type.kind == dronecan_type.KIND_UNSIGNED_INT:
        return False
    elif dronecan_type.kind == dronecan_type.KIND_SIGNED_INT:
        return True
    elif dronecan_type.kind == dronecan_type.KIND_FLOAT:
        return True

def union_msg_tag_bitlen_from_num_fields(num_fields):
    return int(math.ceil(math.log(num_fields,2)))

def union_msg_tag_uint_type_from_num_fields(num_fields):
    return c_uint_type_from_bitlen(union_msg_tag_bitlen_from_num_fields(num_fields))

def array_len_field_bitlen(array_type):
    assert array_type.category == array_type.CATEGORY_ARRAY
    return int(math.ceil(math.log(array_type.max_size+1,2)))

def c_int_type_bitlen(bitlen):
    for ret in (8, 16, 32, 64):
        if bitlen <= ret:
            return ret

def c_uint_type_from_bitlen(bitlen):
    return 'uint%u_t' % (c_int_type_bitlen(bitlen),)

def c_int_type_from_bitlen(bitlen):
    return 'int%u_t' % (c_int_type_bitlen(bitlen),)

def c_array_len_type(field):
    return 'uint%u_t' % (c_int_type_bitlen(array_len_field_bitlen(field.type)),)

def underscored_name_to_ctype(name):
    return 'struct %s' % (name)

def underscored_name_to_cpptype(name):
    return '%s' % (name)

def dronecan_type_to_ctype(dronecan_type):
    assert dronecan_type.category != dronecan_type.CATEGORY_VOID
    if dronecan_type.category == dronecan_type.CATEGORY_COMPOUND:
        assert dronecan_type.kind == dronecan_type.KIND_MESSAGE
        return 'struct %s' % (underscored_name(dronecan_type))
    elif dronecan_type.category == dronecan_type.CATEGORY_PRIMITIVE:
        if dronecan_type.kind == dronecan_type.KIND_BOOLEAN:
            return 'bool'
        elif dronecan_type.kind == dronecan_type.KIND_UNSIGNED_INT:
            return c_uint_type_from_bitlen(dronecan_type.bitlen)
        elif dronecan_type.kind == dronecan_type.KIND_SIGNED_INT:
            return c_int_type_from_bitlen(dronecan_type.bitlen)
        elif dronecan_type.kind == dronecan_type.KIND_FLOAT:
            return 'float' if dronecan_type.bitlen <= 32 else 'double'

def field_cdef(field):
    assert field.type.category != field.type.CATEGORY_VOID
    if field.type.category == field.type.CATEGORY_ARRAY:
        if field.type.mode == field.type.MODE_STATIC:
            return '%s %s[%u]' % (dronecan_type_to_ctype(field.type.value_type), field.name, field.type.max_size)
        else:
            return 'struct { uint%u_t len; %s data[%u]; }%s' % (c_int_type_bitlen(array_len_field_bitlen(field.type)), dronecan_type_to_ctype(field.type.value_type), field.type.max_size, field.name)
    else:
        return '%s %s' % (dronecan_type_to_ctype(field.type), field.name)

def field_get_data(field):
    assert field.type.category == field.type.CATEGORY_ARRAY
    if field.type.mode == field.type.MODE_DYNAMIC:
        return '%s.data' % field.name
    else:
        return '%s' % field.name

def field_flat(fields):
    ret_str = ''
    for field in fields:
        if field.type.category == field.type.CATEGORY_VOID:
            continue
        if field.type.category == field.type.CATEGORY_ARRAY:
            if field.type.mode == field.type.MODE_STATIC:
                ret_str += ', ' + field.name
            else:
                ret_str += ', ' + '%s_len, %s' % (field.name, field.name)
        else:
            ret_str += ', ' + field.name
    return ret_str

def indent(string, n):
    if string.strip():
        string = '    '*n + string
        string.replace('\n', '\n' + '    '*n)
    return string

def msg_header_name_request(obj):
    if isinstance(obj, dronecan.dsdl.Field):
        obj = obj.type
    assert obj.category == obj.CATEGORY_COMPOUND and obj.kind == obj.KIND_SERVICE
    return '%s_req.h' % (obj.full_name,)

def msg_header_name_response(obj):
    if isinstance(obj, dronecan.dsdl.Field):
        obj = obj.type
    assert obj.category == obj.CATEGORY_COMPOUND and obj.kind == obj.KIND_SERVICE
    return '%s_res.h' % (obj.full_name,)

def msg_header_name(obj):
    if isinstance(obj, dronecan.dsdl.Field):
        obj = obj.type
    return '%s.h' % (obj.full_name,)

def msg_c_file_name_request(obj):
    if isinstance(obj, dronecan.dsdl.Field):
        obj = obj.type
    assert obj.category == obj.CATEGORY_COMPOUND and obj.kind == obj.KIND_SERVICE
    return '%s_req.c' % (obj.full_name,)

def msg_c_file_name_response(obj):
    if isinstance(obj, dronecan.dsdl.Field):
        obj = obj.type
    assert obj.category == obj.CATEGORY_COMPOUND and obj.kind == obj.KIND_SERVICE
    return '%s_res.c' % (obj.full_name,)

def msg_c_file_name(obj):
    if isinstance(obj, dronecan.dsdl.Field):
        obj = obj.type
    return '%s.c' % (obj.full_name,)

def msg_test_file_name(obj):
    if isinstance(obj, dronecan.dsdl.Field):
        obj = obj.type
    return 'test_%s.cpp' % (obj.full_name,)

def msg_test_file_name_request(obj):
    if isinstance(obj, dronecan.dsdl.Field):
        obj = obj.type
    return 'test_%s_request.cpp' % (obj.full_name,)

def msg_test_file_name_response(obj):
    if isinstance(obj, dronecan.dsdl.Field):
        obj = obj.type
    return 'test_%s_response.cpp' % (obj.full_name,)

def msg_test_makefile_name(obj):
    if isinstance(obj, dronecan.dsdl.Field):
        obj = obj.type
    return 'test_%s.mk' % (obj.full_name,)

def msg_test_makefile_name_request(obj):
    if isinstance(obj, dronecan.dsdl.Field):
        obj = obj.type
    return 'test_%s_request.mk' % (obj.full_name,)

def msg_test_makefile_name_response(obj):
    if isinstance(obj, dronecan.dsdl.Field):
        obj = obj.type
    return 'test_%s_response.mk' % (obj.full_name,)

def get_canard_src():
    return os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'libcanard/canard.c'))

def get_canard_inc():
    return os.path.dirname(get_canard_src())

def underscored_name(obj):
    return obj.full_name.replace('.','_')

def rel_path(obj):
    return os.path.join(*obj.full_name.split('.')[:-1])

def short_name(obj):
    return obj.full_name.split('.')[-1]

# https://stackoverflow.com/questions/600268/mkdir-p-functionality-in-python
def mkdir_p(path):
    try:
        os.makedirs(path)
    except OSError as exc:  # Python >2.5
        if exc.errno == errno.EEXIST and os.path.isdir(path):
            pass
        else:
            raise

def build_table(msg_underscored_name, msg_union, msg_fields):
    if len(msg_fields) == 0:
        return None # cheaper to encode nothing without a table

    table = _build_table_core(msg_underscored_name, msg_union, msg_fields)
    if table is None: return None

    # search for the last array and mark it eligible for TAO if possible
    array_idx = len(table)-1
    for offset, type_extra, bitlen, *extra in table[::-1]:
        if type_extra == "CANARD_TABLE_CODING_ARRAY_DYNAMIC":
            break
        array_idx -= 1
    if array_idx != -1 and table[array_idx][3] >= 8:
        num_entries = int(table[array_idx][2])
        if num_entries == len(table)-array_idx-3: # this is actually the last thing
            table[array_idx] = (table[array_idx][0], "CANARD_TABLE_CODING_ARRAY_DYNAMIC_TAO", table[array_idx][2])

    return table

def _build_table_core(obj_underscored_name, obj_union, obj_fields):
    if obj_union:
        return None # not yet supported

    table = []
    for field in obj_fields:
        if isinstance(field, dronecan.dsdl.parser.Type):
            t = field
            offset = "0"
        else:
            t = field.type
            offset = f"offsetof(struct {obj_underscored_name}, {field.name})"
        field_name = t.full_name.replace('.', '_').split("[")[0] # hack
        if isinstance(t, dronecan.dsdl.parser.PrimitiveType):
            if t.kind == t.KIND_BOOLEAN or t.kind == t.KIND_UNSIGNED_INT:
                ts = "CANARD_TABLE_CODING_UNSIGNED"
            elif t.kind == t.KIND_SIGNED_INT:
                ts = "CANARD_TABLE_CODING_SIGNED"
            elif t.kind == t.KIND_FLOAT:
                ts = "CANARD_TABLE_CODING_FLOAT"
            table.append((offset, ts, str(t.bitlen)))
        elif isinstance(t, dronecan.dsdl.parser.VoidType):
            table.append(("0", "CANARD_TABLE_CODING_VOID", str(t.bitlen)))
        elif isinstance(t, dronecan.dsdl.parser.CompoundType):
            sub = _build_table_core(field_name, t.union, t.fields) # build the compound table
            if sub is None: return None

            # prepend offset to each entry
            table.extend((f"{offset}+{s[0]}", *s[1:]) for s in sub)
        elif isinstance(t, dronecan.dsdl.parser.ArrayType) and t.mode == t.MODE_STATIC:
            if t.max_size > 256: return None # too big to encode

            if isinstance(t.value_type, dronecan.dsdl.parser.CompoundType):
                sub = _build_table_core(field_name, t.value_type.union, t.value_type.fields)
            else:
                sub = _build_table_core(field_name, False, [t.value_type])
            if sub is None: return None
            if len(sub) == 0: # array of empty objects???
                assert False
            if len(sub) > 255: # too many entries
                return None

            table.append((offset, "CANARD_TABLE_CODING_ARRAY_STATIC", str(len(sub))))
            table.append((f"sizeof({dronecan_type_to_ctype(t.value_type)})", "0", f"{t.max_size}-1"))
            table.extend(sub)
        elif isinstance(t, dronecan.dsdl.parser.ArrayType) and t.mode == t.MODE_DYNAMIC:
            if t.max_size > 65535: return None # too big to encode

            if isinstance(t.value_type, dronecan.dsdl.parser.CompoundType):
                sub = _build_table_core(field_name, t.value_type.union, t.value_type.fields)
            else:
                sub = _build_table_core(field_name, False, [t.value_type])
            if sub is None: return None
            if len(sub) == 0: # array of empty objects???
                assert False
            if len(sub) > 255: # too many entries
                return None

            # stolen from field_cdef
            ddef = 'struct { uint%u_t len; %s data[%u]; }' % (c_int_type_bitlen(array_len_field_bitlen(t)), dronecan_type_to_ctype(t.value_type), t.max_size)
            table.append((f"{offset}+offsetof({ddef}, data)", "CANARD_TABLE_CODING_ARRAY_DYNAMIC", str(len(sub)), t.value_type.get_min_bitlen()))
            table.append((f"sizeof({dronecan_type_to_ctype(t.value_type)})", "0", f"({t.max_size}-1)&0xFF"))
            table.append((f"{offset}+offsetof({ddef}, len)", f"{t.max_size.bit_length()}", f"({t.max_size}-1)>>8"))
            table.extend(sub)
        else:
            assert False # unknown type

    return table
