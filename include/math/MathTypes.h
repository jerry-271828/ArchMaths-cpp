#pragma once

#include <string>
#include <vector>
#include <variant>
#include <functional>
#include <unordered_map>
#include <cmath>
#include <memory>

namespace ArchMaths {

// 数学常量
namespace Constants {
    constexpr double PI = 3.14159265358979323846;
    constexpr double E = 2.71828182845904523536;
    constexpr double PHI = 1.61803398874989484820; // 黄金比例
    constexpr double TAU = 2.0 * PI;
}

// Token类型
enum class TokenType {
    Number,
    Variable,
    Operator,
    Function,
    LeftParen,
    RightParen,
    Comma,
    Equals,
    LessThan,
    GreaterThan,
    LessEqual,
    GreaterEqual,
    End
};

// Token结构
struct Token {
    TokenType type;
    std::string value;
    double numValue = 0.0;

    Token(TokenType t, const std::string& v) : type(t), value(v) {}
    Token(TokenType t, double n) : type(t), numValue(n) {}
};

// 表达式节点类型
enum class NodeType {
    Number,
    Variable,
    BinaryOp,
    UnaryOp,
    Function,
    Conditional
};

// 前向声明
struct ExprNode;
using ExprNodePtr = std::shared_ptr<ExprNode>;

// 表达式节点
struct ExprNode {
    NodeType type;
    double value = 0.0;
    std::string name;
    std::string op;
    ExprNodePtr left;
    ExprNodePtr right;
    std::vector<ExprNodePtr> args;

    static ExprNodePtr makeNumber(double v) {
        auto node = std::make_shared<ExprNode>();
        node->type = NodeType::Number;
        node->value = v;
        return node;
    }

    static ExprNodePtr makeVariable(const std::string& n) {
        auto node = std::make_shared<ExprNode>();
        node->type = NodeType::Variable;
        node->name = n;
        return node;
    }

    static ExprNodePtr makeBinaryOp(const std::string& o, ExprNodePtr l, ExprNodePtr r) {
        auto node = std::make_shared<ExprNode>();
        node->type = NodeType::BinaryOp;
        node->op = o;
        node->left = std::move(l);
        node->right = std::move(r);
        return node;
    }

    static ExprNodePtr makeUnaryOp(const std::string& o, ExprNodePtr operand) {
        auto node = std::make_shared<ExprNode>();
        node->type = NodeType::UnaryOp;
        node->op = o;
        node->left = std::move(operand);
        return node;
    }

    static ExprNodePtr makeFunction(const std::string& n, std::vector<ExprNodePtr> a) {
        auto node = std::make_shared<ExprNode>();
        node->type = NodeType::Function;
        node->name = n;
        node->args = std::move(a);
        return node;
    }
};

// 变量上下文
using VariableContext = std::unordered_map<std::string, double>;

// 函数类型定义
using MathFunction = std::function<double(const std::vector<double>&)>;
using FunctionRegistry = std::unordered_map<std::string, MathFunction>;

// 用户自定义函数
struct UserFunction {
    std::vector<std::string> params;  // 参数名列表
    std::string bodyStr;               // 函数体字符串
};
using UserFunctionRegistry = std::unordered_map<std::string, UserFunction>;

// 绘图类型
enum class PlotType {
    ExplicitY,      // y = f(x)
    ExplicitX,      // x = f(y)
    Implicit,       // f(x,y) = 0
    Parametric2D,   // x=f(t), y=g(t)
    Parametric3D,   // x=f(t), y=g(t), z=h(t)
    Surface3D,      // z = f(x,y)
    Implicit3D,     // f(x,y,z) = 0
    Polar           // r = f(θ)
};

// 颜色结构 (HSB)
struct Color {
    float h = 0.0f;   // 色相 0-360
    float s = 100.0f; // 饱和度 0-100
    float b = 85.0f;  // 亮度 0-100
    float a = 1.0f;   // 透明度 0-1

    // HSB转RGB
    void toRGB(float& r, float& g, float& bl) const {
        float hh = h / 60.0f;
        float ss = s / 100.0f;
        float bb = b / 100.0f;

        int i = static_cast<int>(hh) % 6;
        float f = hh - i;
        float p = bb * (1 - ss);
        float q = bb * (1 - f * ss);
        float t = bb * (1 - (1 - f) * ss);

        switch (i) {
            case 0: r = bb; g = t; bl = p; break;
            case 1: r = q; g = bb; bl = p; break;
            case 2: r = p; g = bb; bl = t; break;
            case 3: r = p; g = q; bl = bb; break;
            case 4: r = t; g = p; bl = bb; break;
            case 5: r = bb; g = p; bl = q; break;
        }
    }
};

// 2D点
struct Point2D {
    double x = 0.0;
    double y = 0.0;

    Point2D() = default;
    Point2D(double x_, double y_) : x(x_), y(y_) {}

    double distanceTo(const Point2D& other) const {
        double dx = x - other.x;
        double dy = y - other.y;
        return std::sqrt(dx * dx + dy * dy);
    }
};

// 3D点
struct Point3D {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    Point3D() = default;
    Point3D(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
};

// 参数信息
struct ParameterInfo {
    std::string name;
    double value = 1.0;
    double minValue = -10.0;
    double maxValue = 10.0;
};

// 绘图条目
struct PlotEntry {
    std::string expression;
    PlotType plotType = PlotType::ExplicitY;
    Color color;
    float thickness = 3.0f;
    bool visible = true;
    bool hasError = false;
    std::string errorMessage;

    // 编译后的表达式树
    ExprNodePtr compiledExpr;
    ExprNodePtr compiledExprX; // 参数方程用
    ExprNodePtr compiledExprY;
    ExprNodePtr compiledExprZ;

    // 参数列表 (非x,y,t,θ的变量)
    std::vector<ParameterInfo> parameters;

    // 缓存的绘图数据
    std::vector<Point2D> plotPoints;
    std::vector<float> vertices; // OpenGL顶点数据

    // 3D绘图数据
    std::vector<Point3D> plotPoints3D;
    std::vector<float> vertices3D;    // x,y,z,nx,ny,nz per vertex
    std::vector<unsigned int> indices3D;  // Triangle indices
};

} // namespace ArchMaths
