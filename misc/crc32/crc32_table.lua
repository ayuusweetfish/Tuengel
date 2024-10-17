local crc32_table = {}
crc32_table[0] = 0
local s = 1
local i = 128
repeat
  local b = (s & 1)
  s = (s >> 1)
  if b ~= 0 then s = s ~ 0xEDB88320 end   -- bit reversed 0x04C11DB7
  for j = 0, 255, 2 * i do
    crc32_table[i + j] = s ~ crc32_table[j]
  end
  i = i >> 1
until i == 0

for i = 0, 255 do io.write(string.format('0x%08x,', crc32_table[i])) end
