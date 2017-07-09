"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class lcmt_contact_info_for_viz(object):
    __slots__ = ["timestamp", "body1_name", "body2_name", "contact_point", "contact_force", "normal"]

    def __init__(self):
        self.timestamp = 0
        self.body1_name = ""
        self.body2_name = ""
        self.contact_point = [ 0.0 for dim0 in range(3) ]
        self.contact_force = [ 0.0 for dim0 in range(3) ]
        self.normal = [ 0.0 for dim0 in range(3) ]

    def encode(self):
        buf = BytesIO()
        buf.write(lcmt_contact_info_for_viz._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">q", self.timestamp))
        __body1_name_encoded = self.body1_name.encode('utf-8')
        buf.write(struct.pack('>I', len(__body1_name_encoded)+1))
        buf.write(__body1_name_encoded)
        buf.write(b"\0")
        __body2_name_encoded = self.body2_name.encode('utf-8')
        buf.write(struct.pack('>I', len(__body2_name_encoded)+1))
        buf.write(__body2_name_encoded)
        buf.write(b"\0")
        buf.write(struct.pack('>3d', *self.contact_point[:3]))
        buf.write(struct.pack('>3d', *self.contact_force[:3]))
        buf.write(struct.pack('>3d', *self.normal[:3]))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != lcmt_contact_info_for_viz._get_packed_fingerprint():
            raise ValueError("Decode error")
        return lcmt_contact_info_for_viz._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = lcmt_contact_info_for_viz()
        self.timestamp = struct.unpack(">q", buf.read(8))[0]
        __body1_name_len = struct.unpack('>I', buf.read(4))[0]
        self.body1_name = buf.read(__body1_name_len)[:-1].decode('utf-8', 'replace')
        __body2_name_len = struct.unpack('>I', buf.read(4))[0]
        self.body2_name = buf.read(__body2_name_len)[:-1].decode('utf-8', 'replace')
        self.contact_point = struct.unpack('>3d', buf.read(24))
        self.contact_force = struct.unpack('>3d', buf.read(24))
        self.normal = struct.unpack('>3d', buf.read(24))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if lcmt_contact_info_for_viz in parents: return 0
        tmphash = (0x8927252e061770f4) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if lcmt_contact_info_for_viz._packed_fingerprint is None:
            lcmt_contact_info_for_viz._packed_fingerprint = struct.pack(">Q", lcmt_contact_info_for_viz._get_hash_recursive([]))
        return lcmt_contact_info_for_viz._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

