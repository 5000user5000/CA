#另外用寫一個hw1-1來確認
#HW1-1 T(n) = 5T(n/2) + 6n + 4, T(1) = 2, round down the result of division



def T(n):
    if n == 1:
        return 2
    else:
        new_n = int(n/2)
        return 5*T(new_n) + 6*n + 4

def main():
    #n = int(input("Enter the value of n: "))
    for i in range(1, 31):
        n = i   
        print(f"T({n}) = ", T(n))  

if __name__ == "__main__":
    main()