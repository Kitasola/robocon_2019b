# robocon_2019b
2019年高専ロボコン「らん♪RUN Laundry」明石高専Bteam「ɒzz!d」   
[AkashiRobo/styleguide](https://github.com/AkashiRobo/styleguide)に則って作成されています.

# しおり

# 運用方法
## 1. Branchについて
[git-flow](https://qiita.com/KosukeSone/items/514dd24828b485c69a05)というモデルがあるのでそれを参考にします.
* masterは常に安定して動くプログラムにする.
* masterから分岐したdevelop/ar(自動機), develop/mr(手動機), develop/cs(コントロールステーション), develop/other(その他)を作成する. developから分岐したブランチで開発を進めていく.
* develop/*hoge*からfeature/*fuga*を作成する. featureで, は様々な機能の開発をする.
* develop/*hoge*からrelease/*fuga*を作成する. releaseでは, featureをdevelopにマージした後の統合やデバッグを行う. releaseで動くことを確認した後, developにマージしてからmasterにマージする. ただし、単機能の場合はreleaseを作らなくても良い.

とします.    
> これは, ロボット間の兼ね合いを考えるのをなるべく少なくしつつ, 1つのリポジトリでBteamで使う全てのプログラムを集約させるためです.

## 2. ディレクトリ構成
### 2.1 ROSのワークスペース
ar/, mr/, cs/がそのままワークスペースとなります.ただし, buildとdevelは.gitignoreに追加します.

### 2.2 MDDのプログラム
ar/mdd_slave/, mr/mdd_slaveの下にディレクトリを作り, 各MDDごとのプログラムを管理します. ただし, 汎用MDDプログラムをそのまま使う場合はディレクトリは作りません.   
mdd_slave/README.mdにMDDのIDと各プログラム(ディレクトリ)との対応を示します. また, IDごとにMDDの各ポートの用途 / 自作CMDの番号と説明なども示します.

## 3. Nucleo開発環境について
gcc-armを使ってコンパイルする.Mbed Onlineからエクスポートできる.
> SW4STM32はGithubとの親和性が低いので禁止

# 使用しているオープンソース
cooming soon...
