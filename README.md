# robocon_2019b
2019年高専ロボコン「らん♪RUN Laundry」明石高専Bteam「ɒzz!d」   
[AkashiRobo/styleguide](https://github.com/AkashiRobo/styleguide)に則って作成されています.

# しおり

# 運用方法
## 1. Branchについて
[git-flow](https://qiita.com/KosukeSone/items/514dd24828b485c69a05)というモデルがあるのでそれを参考にします.
ただし, 
* masterから分岐したmaster/ar, master/mr, master/cs, master/otherを作成する.
* master/*hoge*から分岐したdevelop/*hoge*, feature/*hoge*/*fuga*, release/*hoge*, hotfix/*hoge*, をそれぞれ作成する.

こととします.    
> これは, ロボット間の兼ね合いを考えるのをなるべく少なくしつつ, 1つのリポジトリでBteamで使う全てのプログラムを集約させるためです.

## 2. ROSのワークスペースについて
ar/, mr/, cs/がそのままワークスペースとなります.ただし, buildとdevelは.gitignoreに追加します.

## 3. Nucleo開発環境について
gcc-armを使ってコンパイルする.Mbed Onlineからエクスポートできる.
> SW4STM32はGithubとの親和性が低いので禁止

# 使用しているオープンソース
cooming soon...