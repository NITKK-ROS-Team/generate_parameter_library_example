# generate_parameter_library_example

原文： https://ar-ray.hatenablog.com/entry/2023/07/22/193200

今回は、便利ツールである「generate_parameter_library」について軽く説明します。

2023/07/20の [rosjp LT #51](https://rosjp.connpass.com/event/287546/) で発表した内容の解説です。


<br>

## generate_parameter_libraryとは

generate_parameter_libraryとは、ROS2に対応したパラメータパーサであり、
名前の通り、パラメータを生成します。

何からパラメータを生成するのかというと、YAMLファイルから生成します。

これを使うことで、ソース側で本来定義されるパラメータをYAMLに置き換えることができます。

YAMLで管理できるので、そのままリファレンスになっていたりするので他の人が読みやすいというメリットがります。

<br>

また、このライブラリにはパラメータのミスをYAML側で検知可能だったりします。

ミスをYAMLで検知することで、開発のQOLを向上させて**圧倒的進捗**を獲得することができます。

<br>

ちなみに、開発元はPickNik Roboticsというロボットを開発している企業が開発しており、他の便利ツールもたくさん開発されているので、バージョンが変わったら使えなくなる…ということはないと思います。

[https://picknik.ai](https://picknik.ai)

generate_parameter_libraryは以下のリポジトリにあります。

[https://github.com/PickNikRobotics/generate_parameter_library](https://github.com/PickNikRobotics/generate_parameter_library)

<br>

ここから説明するサンプルコードは以下のリポジトリにあります。

[https://github.com/NITKK-ROS-Team/generate_parameter_library_example](https://github.com/NITKK-ROS-Team/generate_parameter_library_example)

<br>

## パラメータの簡単な定義（YAMLの書き方）

パラメータの定義は、全てyamlで定義されていて、generate_parameter_libraryではyamlファイルを独立したパッケージとして作成して、そのパラメータがほしいrospkg（Node）でインクルードします。

<br>

以下はYAMLの一部です。

`example_params` というパッケージ内に `src/parameter_test_0.yaml` を作成しました。

```yaml
parameter_test:
  data_string: {
    type: string,
    default_value: "string_value",
    description: "test code generation for string",
  }

camera_params:
  width: {
    type: int,
    default_value: 1280,
    description: "test code generation for int",
    validation: {
      one_of<>: [[1280, 960, 848, 640, 424, 352, 320]]
    }
  }
```

yamlのルート（`parameter_test`）がC++の名前空間に当たる部分で、それ以下（`parameter_test`・`camera_params.width`）がパラメータの定義です。

typeがパラメータのタイプ、default_valueがパラメータのデフォルト値となっています。

<br>

`camera_params.width` のようにパラメータを入れ子構造にすることもできます。

あとはCMakeListsとpackage.xmlを作成してビルドするだけで定義完了です。

<br>

次に、ビルドに必要なファイルを示します。

### package.xml

まず、package-xmlです。

↓depends欄をピックアップして説明します。

```xml
<buildtool_depend>ament_cmake_auto</buildtool_depend>

<depend>generate_parameter_library</depend>
<depend>parameter_traits</depend>
<depend>rclcpp_lifecycle</depend>
```

`ament_cmake_auto` は `<depend>` 欄の依存関係を自動で解決できるツールを使用できます。

それ以下の `<depend>` 欄はパラメータ生成に必要なライブラリです。

<br>

### CMakeLists.txt

次に、CMakeLists.txtです。

少し特殊で、 `generate_parameter_library` という構文にYAMLファイルを入れます。

`generate_parameter_library` を呼び出した回数だけヘッダファイルを生成します。

```cmake
cmake_minimum_required(VERSION 3.8)
project(example_params)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(generate_parameter_library REQUIRED)

generate_parameter_library(param_0
  src/parameter_test_0.yaml
)

generate_parameter_library(param_1
  src/parameter_test_1.yaml
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_auto_package()
```

`パラメータ名/パラメータ名.hpp` でパラメータをincludeします。


上記のパターンの場合は以下のヘッダーファイル名が生成されます。

- `param_0/param_0.hpp`
- `param_1/param_1.hpp`


<br>

## パラメータの使用（C++）

generate_parameter_libraryで生成されたパラメータはC++ヘッダーファイルとしてinclude可能です。

> Python向けの生成も可能らしいですが、ここでは説明しません。

```cpp
#pragma once

#include <chrono>

#include "param_0/param_0.hpp"
#include <rclcpp/rclcpp.hpp>

class ParameterExample : public rclcpp::Node
{
public:
    ParameterExample(const rclcpp::NodeOptions &);
protected:
    std::shared_ptr<parameter_test::ParamListener> param_listener_;
    parameter_test::Params params_;
};
```


```cpp
#include "example_cpp/params_test.hpp"

ParameterExample::ParameterExample(const rclcpp::NodeOptions &options)
    : Node("params_test", options)
{
    this->param_listener_ = std::make_shared<parameter_test::ParamListener>(
        this->get_node_parameters_interface()); # 初期化
    this->params_ = param_listener_->get_params(); # パラメータの取得

    RCLCPP_INFO(this->get_logger(), "data_string: %s", this->params_.data_string.c_str()); # パラメータの表示
```

YAMLから生成されたヘッダーファイルは `#include "example_params/example_params.hpp"` にあります。

これをincludeしたら3行でパラメータ取得完了します。

`parameter_test::ParamListener` の初期化には、 `this->get_node_parameters_interface` を渡します。

`RCLCPP_INFO` の部分では、先程YAMLで定義されていた `data_string` がプリントできます。

素晴らしいですね。

<br>

CMakeLists.txtとpackage.xmlは通常のノードの作成と同じ要領で作成します。

package.xmlの `<depends>` 欄にはパラメータ生成pkgの ` <depend>example_params</depend>` を追記しました。

<br>

## 便利なオプション（YAMLの詳細説明）

YAMLでは型とデフォルト値だけでなく、条件式を入れることもできます。

条件式は `validation` キーに入れます。

<br>

### one_of

```yaml
width: {
  type: int,
  default_value: 1280,
  description: "test code generation for int",
  validation: {
    one_of<>: [[1280, 960, 848, 640, 424, 352, 320]]
  }
}
```

`one_of` では、列挙された値の中で指定されていたものだけ通します。

たとえば、カメラのパラメータとかは固定値が決まっているので、この値は通るけど実行エラーになる‥

みたいなことがあると思いますが、これを使うと指定された値以外は通さないようになります。

<br>

### fixed

```yaml
fixed_types:
  fixed_string: {
    type: string_fixed_25,
    default_value: "string_value",
    description: "test code generation for fixed sized string",
  }
  fixed_double_array: {
    type: double_array_fixed_10,
    default_value: [1.0, 2.3, 4.0 ,5.4, 3.3],
    description: "test code generation for fixed sized array",
  }
```

`fixed_` は、可変長な型の制限を設けるものです。

`string_fixed_25` では、25文字以上の文字列をエラーとして弾くことができます。

`default_array_fixed_10` では、10個以上のfloat数列をエラーとして弾きます。

<br>

### lt_eq

```yaml
lt_eq_fifteen: {
  type: int,
  default_value: 1,
  description: "should be a number less than or equal to 15",
  validation: {
    lt_eq<>: [ 15 ],
  }
}
gt_fifteen: {
  type: int,
  default_value: 16,
  description: "should be a number greater than 15",
  validation: {
    gt<>: [ 15 ],
  }
}
```

`lt_eq` は、指定された値より大きいと弾きます。（指定された値以下を指定します）

逆に `gt_eq` は指定された値より小さい場合に弾きます。

<br>

### bounds

```yaml
one_number: {
  type: int,
  default_value: 100,
  validation: {
    bounds<>: [ 0, 1024 ]
  }
}
```

`bounds` は値の範囲を指定して、そこから外れたら弾くようにします。

つまり、指定された値以上。値以下を指定しています。

<br>

### read_only

rosパラメータの動的変更を禁止する `read_only` キーワードは、ノード起動時以外のパラメータ設定を禁止します。

```yaml
read_only:
  data_int: {
    type: int,
    default_value: 1,
    read_only: true,
  }
```

![image](https://github.com/NITKK-ROS-Team/generate_parameter_library_example/assets/67567093/7bf095b8-0387-4a2e-8814-99a89a14b6f2)

rqt_reconfigureにも表示されません。

![image](https://github.com/NITKK-ROS-Team/generate_parameter_library_example/assets/67567093/308c3289-4cfc-43cf-aa1b-929c02528aaa)

<br>

### map

また、C++におけるdictionaryのような形で指定できる `map` キーワードも用意されています。

```yaml
data_double: {
  type: string_array,
  default_value: ["a", "b", "c"],
  description: "test code generation for string array",
}
test_map:
  __map_data_double:
    data_double_0: {
      type: double,
      default_value: .NAN,
      description: "data double 0"
    }
    data_double_1: {
      type: double,
      default_value: 0.5,
      description: "data double 1"
    }
    data_double_2: {
      type: double,
      default_value: .NAN,
      description: "data double 2"
    }
```

<br>

ちょっと複雑めな感じのパラメータ定義もYAMLで楽に設定できるので、おすすめです。
