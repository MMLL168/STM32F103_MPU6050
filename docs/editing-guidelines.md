# 專案修改邊界

## 可以直接修改的檔案

- `src/`
- `include/`
- `lib/`
- `platformio.ini`
- `docs/`
- `devlog.md`

這些屬於專案自己的內容，正常功能開發、除錯、重構都應該優先改這裡。

## 盡量不要直接修改的檔案

- `.pio/`
- `.vscode/launch.json`
- `C:\Users\marlonwu\.platformio\packages\...`
- `C:\Users\marlonwu\.platformio\platforms\...`

原因：
- `.pio/` 是編譯輸出目錄，會被重建覆蓋。
- `.vscode/launch.json` 多半由 PlatformIO 自動產生，可能被重新生成。
- `.platformio\packages` 與 `.platformio\platforms` 是 PlatformIO 套件區，更新、重裝或清除快取後就可能消失。

## 如果真的需要改 framework

優先順序建議如下：

1. 先找 `build_flags`、`lib_deps`、`board_build.*` 能不能解決。
2. 如果只是要覆蓋邏輯，優先在 `src/` 或 `lib/` 內包一層自己的程式，不要直接改套件原檔。
3. 如果真的要改 framework 原始碼：
   - 先記錄到 `devlog.md`
   - 記下原始路徑
   - 記下修改原因
   - 記下如何復原
4. 若修改會長期保留，建議把需要的檔案複製進專案內做本地 override，而不是依賴使用者目錄中的套件檔。

## 這個專案的實務建議

- IMU、PID、PWM、RC 解碼：改 `src/main.cpp` 或拆到 `lib/`
- 編譯、燒錄、除錯設定：改 `platformio.ini`
- 文件、接線、除錯流程：改 `docs/`
- 不要把正式功能邏輯寫進 `.platformio\packages\framework-arduinoststm32\...`

## 遇到套件檔異常時

若懷疑是 PlatformIO framework 套件被改壞：

1. 先確認錯誤是否來自專案檔還是套件檔
2. 若是套件檔，先做最小修復讓專案恢復可編譯
3. 再把修復記錄到 `devlog.md`
4. 後續再決定是否要重裝 PlatformIO 套件，避免同樣問題再次出現
