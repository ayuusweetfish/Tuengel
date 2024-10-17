import zlib
a = '123456789'
s = zlib.crc32(a) & 0xffffffff
print('%08x' % s)
