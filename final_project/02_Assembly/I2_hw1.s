.data
    n: .word 10
    
.text
.globl __start

# Todo: Define your own function in HW1
# You should store the output into x10
FUNCTION:
  # 計算 T(n) 所需的初始化
  addi t4, x0, 0     # 初始化 t4 為 0
  add  t1, x0, x10    # 把 n 的值暫存於 t1，在recurse會不斷除以2
  addi a2, x0, 5     # 初始化 a2 為 5，T(n/2)的係數
  addi a3, x0, 6     # 初始化 a3 為 6，n的係數
  addi a5, x0, 1     # 初始化 a5 為 1，用來當迴圈判定

  jal t6,recurse         # 呼叫遞迴函式，把當前位置存到 t6(ra)
  addi x10, t4 , 0   # 把結果存到 x10
  jr x1          # 直接跳到result


recurse:
  addi sp,sp,-8    # 遞迴函式，先把 $sp 減8，因為要存 t6(ra) 和 參數，改用 t6，因為 ra==x1 會被覆蓋
  sw t6,4(sp)       # 把 t6(ra) 存到 $sp+4 的位置
  sw t1,0(sp)       # 把 t1 存到 $sp+0 的位置

  beq a5, t1, end    # 如果 n == 1，就跳到 end

  srli t1, t1, 1     # 計算 n/2，向右邊移位 1 位，將值存入 t1。 n 應該會是正數，所以就不用 srai。其本身是無條件捨去法。
  jal t6,recurse        

  lw t1,0(sp)       # 把 當前的n值 從 $sp+0 的位置取出
  mul t4,t4,a2       # 5T(n/2) 
  mul t2,a3,t1       # 6n 
  add t4,t4,t2       # 5T(n/2) + 6n
  addi t4,t4,4       # 5T(n/2) + 6n + 4

  lw t6,4(sp)       # 把 t6(ra) 從 $sp+4 的位置取出
  addi sp,sp,8      # 把 $sp 加8，pop
  jr   t6

end:
  addi t4, x0, 2    # 如果 n == 1，就把 t4 設為 2
  lw t6,4(sp)       # 把 t6(ra) 從 $sp+4 的位置取出
  addi sp,sp,8      # 把 $sp 加8，pop
  jr   t6
    

# Do NOT modify this part!!!
__start:
    la   t0, n
    lw   x10, 0(t0)
    jal  x1,FUNCTION
    la   t0, n
    sw   x10, 4(t0)
    addi a0,x0,10
    ecall