# hw1-1

使用 RISC-V 的組合語言來寫 <br>
`T(n) = 5T(n/2) + 6n + 4, T(1) = 2 `, round down the result of division <br>
用遞迴的方式解 T(n)的值 <br>
要看原始的作業，可以用 git history 到 first commit <br>
解法很簡單: T(n/2)先不要算值是多少，先把 6n+4 累加到 t0 ，每一次遞迴 n 都會除以二(無條件捨去小數)，且乘在 6n+4 的 t3 則是每一次都會增加 5 倍 (1、5、25...)，因為每次遞迴子問題都會乘上 5 倍。 T(n) = 5T(n/2) + 6n + 4 = 25\*T(n/4) + (6n+4) + 5\*(6\*n/2+4) = ... <br>
為了測試是否正確，特別寫了一個 hw1-1.py，會生成 n = 1~30 的值，就自己找幾個例子輸入看看。edge case n=1 也要測試好。 <br>

# hw1-2

輸入 shift 和 plaintext，經由凱薩加密得到 Ciphertext。 <br>
空白是轉成數字，從 0 開始記數，最多到 9。 <br>
輸入內容只限小寫英文字母和空白鍵，shift 則是 -12~13。 <br>
Plaintext will end with ‘\n’ (decimal 10) <br>

例如:

```
shift = 3
panintext = “abc and cde”
Ciphertext = “def0dqg1fgh”
```

這題的解法是要寫一個 loop，不斷從 0x10200 取字符，取到後做 shift 處理，因為可能超過 a-z 的邊界，以及空白的時候，所以要做 3 個條件式，跳出去 loop 去處理。
