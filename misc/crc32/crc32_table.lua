local crc32_table = {}
crc32_table[0] = 0
local s = 1 << 31
local i = 1
repeat
  local b = (s & (1 << 31))
  s = (s << 1) & ((1 << 32) - 1)
  if b ~= 0 then s = s ~ 0x04C11DB7 end
  for j = 0, i - 1 do
    crc32_table[i + j] = s ~ crc32_table[j]
  end
  i = i << 1
until i >= 256

for i = 0, 255 do io.write(string.format('0x%08x,', crc32_table[i])) end
