# bringme_action:  
## 概要
２章のサンプルプログラム  
airobot_interfacesを使ったアクション通信のサンプルパッケージ


## インストール
Chapter2のパッケージは全部まとめてインストール・ビルドをします．
- [第2章 インストール](https://github.com/AI-Robot-Book/chapter2)を参照してください．


## 実行  
- 端末を２つに分割する．
- 1番目の端末で次のコマンドを実行する．  
```
ros2 run bringme_action bringme_action_server_node  
```
- 2番目の端末で次のコマンドを実行する．
```
ros2 run bringme_action bringme_action_client_node
何をとってきますか：
```
と聞かれるので，取ってきて欲しい英単語を入力する．  
'apple', 'banana', 'candy'のいずれかを入力すると以下のように表示されます．  
[INFO] [1728888853.238853119] [bringme_action_client]: ゴールが承認されました  
[INFO] [1728888857.243842144] [bringme_action_client]: フィードバック受信中: 残り5[s]  
[INFO] [1728888858.245358757] [bringme_action_client]: フィードバック受信中: 残り4[s]  
[INFO] [1728888859.247032373] [bringme_action_client]: フィードバック受信中: 残り3[s]  
[INFO] [1728888860.248677167] [bringme_action_client]: フィードバック受信中: 残り2[s]  
[INFO] [1728888861.250357551] [bringme_action_client]: フィードバック受信中: 残り1[s]  
[INFO] [1728888862.252739181] [bringme_action_client]: ゴールの結果: はい，〇〇 (apple/banana/candy) です．  

それ以外は，次のように表示されます．  
[INFO] [1728889050.299714176] [bringme_action_client]: ゴールが承認されました  
[INFO] [1728889051.292081616] [bringme_action_client]: フィードバック受信中: 残り5[s]  
[INFO] [1728889052.293734543] [bringme_action_client]: フィードバック受信中: 残り4[s]  
[INFO] [1728889053.295324400] [bringme_action_client]: フィードバック受信中: 残り3[s]  
[INFO] [1728889054.296944174] [bringme_action_client]: フィードバック受信中: 残り2[s]  
[INFO] [1728889055.298635793] [bringme_action_client]: フィードバック受信中: 残り1[s]  
[INFO] [1728889056.300734920] [bringme_action_client]: ゴールの結果: orangeを見つけることができませんでした．

## ヘルプ
- 今のところありません．
　
 
## 著者
升谷 保博，出村 公成


## 履歴
- 2024-10-14: 初期版


## ライセンス
Copyright (c) 2025,Yasuhiro Masutani and Kosei Demura, All rights reserved. This project is licensed under the Apache-2.0 license found in the LICENSE file in the root directory of this project.


## 参考文献
- 今のところありません．

