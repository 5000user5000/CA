# 凱薩加密，輸入plaintext和shift，輸出ciphertext
# 空白則是用數字表示，從0開始遞增



def caesar_cipher(plaintext, shift):
    ciphertext = ""
    space = 0
    for i in plaintext:
        if i == " ":
            ciphertext += str(space)
            space += 1
        else:
            if(ord(i) + shift > 122):
                ciphertext += chr(ord(i) + shift - 26)
            elif(ord(i) + shift  < 97):
                ciphertext += chr(ord(i) + shift + 26)
            else:
               ciphertext += chr(ord(i) + shift)
    return ciphertext



if __name__ == "__main__":
    shift = int(input("Shift: "))
    plaintext = input(" Plaintext: ")
    print("Ciphertext: " + caesar_cipher(plaintext, shift))
