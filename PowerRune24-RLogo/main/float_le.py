# 循环调用，生成小端浮点变量的HEX字符串
# 输入：浮点数
# 输出：HEX字符串

import struct
import binascii

def float_le(f):
    return binascii.hexlify(struct.pack('<f', f)).decode('utf-8')

if __name__ == '__main__':
    while True:
        print(float_le(float(input('Please input a float number: '))))