# CG Physics Simulation Study (Unity)

本リポジトリは、Unity（URP）環境において物理シミュレーション手法を学習・再現することを目的とした個人制作プロジェクトです。

書籍に記載された数式およびアルゴリズムを単に写経するのではなく、各ステップの数学的・物理的意味を理解することを重視して実装しています。

---

## 📚 参考文献

本プロジェクトは、以下の書籍を参考に制作しています。

- **『CGのための物理シミュレーションの基礎』**

書籍内で解説されている数式およびアルゴリズムを基に、Unity（URP）環境で動作する形に再構築しています。

---

## 🧮 使用ライブラリ

本プロジェクトでは数値計算のために **Math.NET Numerics** を使用しています。

This project includes Math.NET Numerics,  
licensed under the MIT License.

Copyright (c) Math.NET  
https://numerics.mathdotnet.com/

---

## 🛠 開発環境

- Unity 2022.3.x（URP）
- C#
- Math.NET Numerics

---

## 📂 再現方法

1. 本リポジトリをクローン
2. Unity Hub からプロジェクトを開く
3. 対象シーンを開いて再生

※ 必要に応じて `Packages` または `Plugins` 内の Math.NET Numerics を確認してください。

---

## 🎯 目的

- 物理シミュレーション理論の理解
- 数値積分・剛体運動・制約処理などの実装理解
- Unity 環境における数値計算の実践
- グラフィックスおよび物理分野の専門性向上
