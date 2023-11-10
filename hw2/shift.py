# 用來模擬硬體做乘法和除法運算

def binary_multiplier(a, b):
    # 初始化 64 位元的 c
    c = [0] * 64

    print(f"Initial a: {a}")
    print(f"Initial b: {b}")

    # 將 b 放到 c 的右半邊
    c[32:] = [int(bit) for bit in b]

    print("Multiplication process:")
    for i in range(32):
        print(f"Step {i + 1}:")
        #print(f"    a: {a}")
        #print(f"    b: {b}")
        print(f"    c: {  hex(int(''.join(map(str, c)),2))  }")

        # 如果 c 最右邊的 bit 為 1，將 a 加到 c 的左半邊
        carry = 0
        if c[-1] == 1:
            carry = 0
            for j in range(31,-1,-1): # 得反向遍歷
                a_bit = int(a[j])
                b_bit = int(c[j])
                # 進行加法
                c[j] = (a_bit ^ b_bit ^ carry)
                # 計算進位
                carry = (a_bit & b_bit) | (carry & (a_bit ^ b_bit))
            print(f"        Add a to the left half of c")

        # 將 c 向右 shift 1 bit
        c = [0] + c[:-1]
        if(carry == 1):
            c[0] = 1

    print("\nFinal result:")
    # print(f"    a: {hex(int(a,2))}")
    # print(f"    b: {hex(int(b,2))}")
    print(f"    c: {  hex(int(''.join(map(str, c)),2))  }")

def compare(a, b):
    for i in range(32):
        if(int(a[i]) > int(b[i])):
            return True
        elif(int(a[i]) < int(b[i])):
            return False
    return True
def binary_division(a, b):
    # 初始化 64 位元的 c
    c = [0] * 64
    b = [0] * (32-len(b)) + [int(bit) for bit in b]
    a = [0] * (32-len(a)) + [int(bit) for bit in a]
    #print(len(b))
    print(f"Initial a: {a}")
    print(f"Initial b: {b}")

    # 將 a 放到 c 的右半邊
    c[32:] = [int(bit) for bit in a]

    print("Division process:")
    for i in range(33):
        print(f"Step {i + 1}:")
        #print(f"    a: {a}")
        #print(f"    b: {b}")
        print(f"    c: {  hex(int(''.join(map(str, c)),2))  }")

        # 如果 c 最右邊的 bit 為 1，將 a 加到 c 的左半邊
        valid_shift = compare(c[0:32], b)
        if valid_shift:
            print("got valid shift")
            for j in range(31,-1,-1):
                #print(f"j: {j}")
                temp = c[j] - b[j]
                if(temp < 0):
                    c[j] = temp + 2
                    c[j-1] = c[j-1] - 1
                else:
                    c[j] = temp

        # 將 c 向左 shift 1 bit
        c = c[1:] + [0]
        if valid_shift:
            c[-1] = 1
    c = [0] + c[0:31] + c[32:64]
    print(c[0:31])
    print(c[32:64])
    print("\nFinal result:")
    # print(f"    a: {hex(int(a,2))}")
    # print(f"    b: {hex(int(b,2))}")
    print(f"    c: {  hex(int(''.join(map(str, c)),2))  }")
    # ans == "0x0297904900000005"

# 測試
a = int("0x0fc93c4a" , 16)
b = int("0x02a388cd", 16)
# a = int("0x00000008" , 16)
# b = int("0x00000002", 16)
a = bin(a)[2:]
b = bin(b)[2:]
# a = int("0x00000007" , 16)
# b = int("0x00000005", 16)
# a = [bit for bit in (29*'0' + bin(a)[2:])]
# b = [bit for bit in (29*'0' + bin(b)[2:])]

#binary_multiplier(a, b)
binary_division(a, b)