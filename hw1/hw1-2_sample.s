.globl __start

.rodata
    msg0: .string "This is HW1-2: \n"
    msg1: .string "Enter shift: "
    msg2: .string "Plaintext: "
    msg3: .string "Ciphertext: "
.text

################################################################################
  # print_char function
  # Usage: 
  #     1. Store the beginning address in x20
  #     2. Use "j print_char"
  #     The function will print the string stored from x20 
  #     When finish, the whole program with return value 0

print_char:
    addi a0, x0, 4
    la a1, msg3
    ecall
  
    add a1,x0,x20
    ecall

  # Ends the program with status code 0
    addi a0,x0,10
    ecall
    
################################################################################

__start:
  # Prints msg
    addi a0, x0, 4
    la a1, msg0
    ecall

  # Prints msg1
    addi a0, x0, 4
    la a1, msg1
    ecall
  # Reads an int
    addi a0, x0, 5
    ecall
    add a6, a0, x0
    
  # Prints msg2
    addi a0, x0, 4
    la a1, msg2
    ecall
    
    addi a0,x0,8
    li a1, 0x10150
    addi a2,x0,2047
    ecall
  # Load address of the input string into a0
    add a0,x0,a1


################################################################################ 
  # Write your main function here. 
  # a0 stores the begining Plaintext
  # x16 stores the shift
  # Do store 66048(0x10200) into x20 
  # ex. j print_char
  
main:
    # Initialize x20 to the start of the ciphertext buffer (0x10200)
    li x20, 0x10200
    addi s0, x0, 48     # s0 = 0，用來當 space 計數器。ascii 48 = '0'

    addi t1, x0, 10
    addi t2, x0, 32
    addi t3, x0, 97
    addi t4, x0, 123
    # Loop through the plaintext string
loop:
    lbu s1, 0(a0)         # Load a character from plaintext
    beqz s1, done         # 如果是 0 就跳到 done
    beq s1, t1, done     # 如果是 newline (code=10)，代表結束，也跳到 done

    beq s1, t2, space     # 如果是空格就跳到 space 去處理

    add s1, s1, a6      # shift the character
    bge s1, t4, bigger  # 如果shift後的值大於等於123，就到bigger去計算
    blt s1, t3, smaller # 如果shift後的值小於97，就到smaller去計算

    sb s1, 0(x20)        # 儲存加密文字到 ciphertext buffer
    addi x20, x20, 1     # Increment the ciphertext buffer pointer
    addi a0, a0, 1       # Increment the plaintext buffer pointer
    j loop               # Repeat the loop

space:   # 如果是空格就用計數值
    sb s0, 0(x20)        # 把 space 計數值存到 ciphertext buffer
    addi x20, x20, 1    
    addi a0, a0, 1       
    addi s0, s0, 1       # Increment the space counter
    j loop               # Repeat the loop

bigger:
    addi s1, s1, -26       # shift後超過邊界(z=122)多少，之後又因為a=97，所以 超過的值+96 = shift後的值。 s1-122+96 = s1-26     
    sb s1, 0(x20)        
    addi x20, x20, 1     
    addi a0, a0, 1       
    j loop               # Repeat the loop

smaller:
    addi s1, s1, 26      # shift後超過邊界(a=97)多少，會是負數，z=122，所以 超過的值+123 = shift後的值。s1-97+123=s1+26
    sb s1, 0(x20)        
    addi x20, x20, 1     
    addi a0, a0, 1       
    j loop               # Repeat the loop



done:
    # 在結尾加上 null，這樣 print_char 才知道結尾
    sb x0, 0(x20)

    # 把 x20 設回 0x10200
    li x20, 0x10200

    # Call the print_char function to print the ciphertext
    j print_char


################################################################################

