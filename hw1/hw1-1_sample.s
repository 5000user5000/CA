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
  addi t3, x0, 1     # 初始化 t3 為 1，5的倍數
  addi a2, x0, 5     # 初始化 a2 為 5，T(n/2)的係數
  addi a3, x0, 6     # 初始化 a3 為 6，n的係數
  addi a4, x0, 2     # 初始化 a4 為 2，也就是 T(1) = 2
  addi a5, x0, 1     # 初始化 a5 為 1，用來當迴圈判定

  #處理 n=1 的case
  bne a0,t3,recurse  # a0 != 1 的話，就到recurse
  addi t0,x0,2       # T(1) = 2
  j result           # 直接跳到result

recurse:
  mul t2, t1, a3     # 計算 6*n，這裡的n會隨著每次遞迴除2
  addi t2, t2, 4     # 計算 6n+4
  mul t2,t2,t3       # 計算 (6n+4)*(5^x)，x=0,1,2...
  mul t3,t3,a2       # t3每次都增加5倍
  add t0, t0, t2     # 把本次結果累加在t0
  srli t1, t1, 1     # 計算 n/2，向右邊移位 1 位，將值存入 t1。 n 應該會是正數，所以就不用 srai
  bne t1, a5 , recurse      # 如果t1不等於1就繼續遞迴。
  mul t4,t3,a4       #最後一輪，把 T(1)*(5^k) 結果算出，並在下一行加入t0
  add t0,t0,t4       #不需要特別去四捨五入，每次遞迴的N都是整數，相乘相加的係數都是整數


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