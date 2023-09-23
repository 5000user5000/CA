.globl __start

.rodata
    msg0: .string "This is HW1-1: T(n) = 5T(n/2) + 6n + 4, T(1) = 2\n"
    msg1: .string "Enter a number: "
    msg2: .string "The result is: "

.text


__start:
  # Prints msg0
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


  # Write your main function here. 
  # Input n is in a0. You should store the result T(n) into t0
  # HW1-1 T(n) = 5T(n/2) + 6n + 4, T(1) = 2, round down the result of division
  # ex. addi t0, a0, 1
initial:
  # 計算 T(n) 所需的初始化
  addi t0, x0, 0     # 初始化 t0 為 0
  add  t1, x0, a0    # 把 n 的值暫存於 t1，在recurse會不斷除以2
  addi a2, x0, 5     # 初始化 a2 為 5，T(n/2)的係數
  addi a3, x0, 6     # 初始化 a3 為 6，n的係數
  addi a5, x0, 1     # 初始化 a5 為 1，用來當迴圈判定

  jal recurse         # 呼叫遞迴函式
  j result           # 直接跳到result


recurse:
  addi sp,sp,-8    # 遞迴函式，先把 $sp 減8，因為要存 ra 和 參數
  sw ra,4(sp)       # 把 ra 存到 $sp+4 的位置
  sw t1,0(sp)       # 把 t1 存到 $sp+0 的位置

  beq a5, t1, end    # 如果 n == 1，就跳到 end

  srli t1, t1, 1     # 計算 n/2，向右邊移位 1 位，將值存入 t1。 n 應該會是正數，所以就不用 srai。其本身是無條件捨去法。
  jal recurse        

  lw t1,0(sp)       # 把 當前的n值 從 $sp+0 的位置取出
  mul t0,t0,a2       # 5T(n/2) 
  mul t2,a3,t1       # 6n 
  add t0,t0,t2       # 5T(n/2) + 6n
  addi t0,t0,4       # 5T(n/2) + 6n + 4

  lw ra,4(sp)       # 把 ra 從 $sp+4 的位置取出
  addi sp,sp,8      # 把 $sp 加8，pop
  jr   ra

end:
  addi t0, x0, 2    # 如果 n == 1，就把 t0 設為 2
  lw ra,4(sp)       # 把 ra 從 $sp+4 的位置取出
  addi sp,sp,8      # 把 $sp 加8，pop
  jr   ra

result:
  # Prints msg2
    addi a0, x0, 4
    la a1, msg2
    ecall

  # Prints the result in t0
    addi a0, x0, 1
    add a1, x0, t0
    ecall
    
  # Ends the program with status code 0
    addi a0, x0, 10
    ecall