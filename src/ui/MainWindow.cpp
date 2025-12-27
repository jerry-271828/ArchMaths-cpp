#include "ui/MainWindow.h"
#include "ui/SidePanel.h"
#include <QMenuBar>
#include <QToolBar>
#include <QStatusBar>
#include <QFileDialog>
#include <QMessageBox>
#include <QDebug>
#include <cmath>
#include <sstream>

namespace ArchMaths {

// 简单的函数定义解析 (不使用regex)
static bool parseFunctionDefinition(const std::string& expr, std::string& funcName,
                                     std::vector<std::string>& params, std::string& body) {
    // 查找 name(params) = body 模式
    size_t parenOpen = expr.find('(');
    if (parenOpen == std::string::npos || parenOpen == 0) return false;

    size_t parenClose = expr.find(')', parenOpen);
    if (parenClose == std::string::npos) return false;

    size_t eqPos = expr.find('=', parenClose);
    if (eqPos == std::string::npos) return false;

    // 提取函数名
    funcName = expr.substr(0, parenOpen);
    // 去除空格
    funcName.erase(std::remove_if(funcName.begin(), funcName.end(), ::isspace), funcName.end());

    // 检查函数名是否有效 (只包含字母数字下划线，以字母开头)
    if (funcName.empty() || !std::isalpha(funcName[0])) return false;
    for (char c : funcName) {
        if (!std::isalnum(c) && c != '_') return false;
    }

    // 转小写
    std::transform(funcName.begin(), funcName.end(), funcName.begin(), ::tolower);

    // 提取参数
    std::string paramsStr = expr.substr(parenOpen + 1, parenClose - parenOpen - 1);
    std::stringstream ss(paramsStr);
    std::string param;
    while (std::getline(ss, param, ',')) {
        param.erase(std::remove_if(param.begin(), param.end(), ::isspace), param.end());
        std::transform(param.begin(), param.end(), param.begin(), ::tolower);
        if (!param.empty()) {
            params.push_back(param);
        }
    }

    // Validate each parameter is a valid identifier (not an expression like "x*y")
    for (const auto& p : params) {
        if (p.empty() || !std::isalpha(p[0])) return false;
        for (char c : p) {
            if (!std::isalnum(c) && c != '_') return false;
        }
    }

    // 提取函数体
    body = expr.substr(eqPos + 1);
    // 去除前导空格
    size_t start = body.find_first_not_of(" \t");
    if (start != std::string::npos) {
        body = body.substr(start);
    }

    return !body.empty();
}

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
    , parser_(std::make_unique<ExpressionParser>())
    , evaluator_(std::make_unique<ExpressionEvaluator>())
{
    parser_->setUserFunctions(&userFunctions_);
    setupUI();
    setupMenuBar();
    setupToolBar();
    connectSignals();

    // 设置默认窗口大小 - Updated
    resize(1200, 800);
    setWindowTitle("Arch Maths - Updated Version");
}

MainWindow::~MainWindow() = default;

void MainWindow::setupUI() {
    splitter_ = new QSplitter(Qt::Horizontal, this);

    // 侧边栏
    sidePanel_ = new SidePanel(this);
    sidePanel_->setMinimumWidth(250);
    sidePanel_->setMaximumWidth(400);

    // 画布
    canvas_ = new GLCanvas(this);

    splitter_->addWidget(sidePanel_);
    splitter_->addWidget(canvas_);
    splitter_->setStretchFactor(0, 0);
    splitter_->setStretchFactor(1, 1);

    setCentralWidget(splitter_);

    // 状态栏
    statusBar()->showMessage("就绪");
}

void MainWindow::setupMenuBar() {
    QMenu* fileMenu = menuBar()->addMenu("文件(&F)");

    QAction* newAction = fileMenu->addAction("新建(&N)");
    newAction->setShortcut(QKeySequence::New);
    connect(newAction, &QAction::triggered, this, [this]() {
        entries_.clear();
        sidePanel_->clear();
        canvas_->setPlotEntries({});
    });

    QAction* openAction = fileMenu->addAction("打开(&O)");
    openAction->setShortcut(QKeySequence::Open);

    QAction* saveAction = fileMenu->addAction("保存(&S)");
    saveAction->setShortcut(QKeySequence::Save);

    fileMenu->addSeparator();

    QAction* exitAction = fileMenu->addAction("退出(&X)");
    exitAction->setShortcut(QKeySequence::Quit);
    connect(exitAction, &QAction::triggered, this, &QMainWindow::close);

    // 视图菜单
    QMenu* viewMenu = menuBar()->addMenu("视图(&V)");

    QAction* resetViewAction = viewMenu->addAction("重置视图(&R)");
    connect(resetViewAction, &QAction::triggered, this, [this]() {
        canvas_->setOffset(QPointF(width() / 2, height() / 2));
        canvas_->setScale(50.0);
    });

    // 帮助菜单
    QMenu* helpMenu = menuBar()->addMenu("帮助(&H)");

    QAction* aboutAction = helpMenu->addAction("关于(&A)");
    connect(aboutAction, &QAction::triggered, this, [this]() {
        QMessageBox::about(this, "关于 Arch Maths",
            "Arch Maths - 高性能数学绘图工具\n\n"
            "基于 Qt 和 OpenGL 构建\n"
            "版本 1.0.0");
    });
}

void MainWindow::setupToolBar() {
    QToolBar* toolbar = addToolBar("工具栏");

    QAction* addAction = toolbar->addAction("添加");
    connect(addAction, &QAction::triggered, this, &MainWindow::onAddEntry);

    toolbar->addSeparator();

    QAction* zoomInAction = toolbar->addAction("放大");
    connect(zoomInAction, &QAction::triggered, this, [this]() {
        if (canvas_->is3DMode()) {
            canvas_->zoom3D(0.8f);
        } else {
            canvas_->setScale(canvas_->getScale() * 1.2);
            recalculateAll();
        }
    });

    QAction* zoomOutAction = toolbar->addAction("缩小");
    connect(zoomOutAction, &QAction::triggered, this, [this]() {
        if (canvas_->is3DMode()) {
            canvas_->zoom3D(1.25f);
        } else {
            canvas_->setScale(canvas_->getScale() / 1.2);
            recalculateAll();
        }
    });

    toolbar->addSeparator();

    QAction* toggle3DAction = toolbar->addAction("3D");
    toggle3DAction->setCheckable(true);
    connect(toggle3DAction, &QAction::toggled, this, [this](bool checked) {
        canvas_->set3DMode(checked);
        recalculateAll();
    });

    QAction* panModeAction = toolbar->addAction("Pan");
    panModeAction->setCheckable(true);
    panModeAction->setEnabled(false);
    connect(panModeAction, &QAction::toggled, canvas_, &GLCanvas::set3DPanMode);
    connect(toggle3DAction, &QAction::toggled, panModeAction, &QAction::setEnabled);
}

void MainWindow::connectSignals() {
    connect(sidePanel_, &SidePanel::addEntryRequested,
            this, &MainWindow::onAddEntry);

    connect(sidePanel_, &SidePanel::entryChanged,
            this, &MainWindow::onEntryChanged);

    connect(sidePanel_, &SidePanel::entryDeleted,
            this, &MainWindow::onEntryDeleted);

    connect(sidePanel_, &SidePanel::entryParameterChanged,
            this, &MainWindow::onParameterChanged);

    connect(sidePanel_, &SidePanel::precisionChanged,
            this, &MainWindow::setPrecisionMultiplier);

    connect(sidePanel_, &SidePanel::entryVisibilityChanged,
            this, &MainWindow::onEntryVisibilityChanged);

    connect(sidePanel_, &SidePanel::entryColorChanged,
            this, &MainWindow::onEntryColorChanged);

    connect(canvas_, &GLCanvas::viewChanged,
            this, &MainWindow::onViewChanged);

    connect(canvas_, &GLCanvas::view3DChanged,
            this, &MainWindow::recalculateAll);

    connect(canvas_, &GLCanvas::mousePositionChanged,
            this, [this](QPointF pos) {
        statusBar()->showMessage(QString("x: %1, y: %2")
            .arg(pos.x(), 0, 'f', 4)
            .arg(pos.y(), 0, 'f', 4));
    });
}

void MainWindow::onAddEntry() {
    PlotEntry entry;
    entry.expression = "";
    entry.color = Color{
        static_cast<float>(std::fmod(entries_.size() * 137.5, 360.0)),
        100.0f, 85.0f, 1.0f
    };

    entries_.push_back(entry);
    sidePanel_->addEntry(entry);
}

void MainWindow::onEntryChanged(int index, const QString& expression) {
    if (index < 0 || index >= static_cast<int>(entries_.size())) return;

    entries_[index].expression = expression.toUtf8().constData();

    // 重新解析所有函数定义
    userFunctions_.clear();
    for (size_t i = 0; i < entries_.size(); ++i) {
        std::string funcName;
        std::vector<std::string> params;
        std::string bodyStr;
        if (parseFunctionDefinition(entries_[i].expression, funcName, params, bodyStr)) {
            // Only register if function doesn't already exist (first definition wins)
            if (userFunctions_.count(funcName) == 0) {
                UserFunction func;
                func.params = std::move(params);
                func.bodyStr = std::move(bodyStr);
                userFunctions_[funcName] = std::move(func);
                qDebug() << "Registered function:" << QString::fromStdString(funcName)
                         << "with body:" << QString::fromStdString(userFunctions_[funcName].bodyStr);
            }
        }
    }
    qDebug() << "Total user functions:" << userFunctions_.size();

    // 现在解析当前entry
    try {
        qDebug() << "Parsing entry:" << index;
        parseAndCompileEntry(entries_[index]);
        qDebug() << "Entry parsed, hasError:" << entries_[index].hasError;

        if (!entries_[index].hasError && entries_[index].compiledExpr) {
            qDebug() << "Extracting parameters";
            extractParameters(entries_[index]);
            qDebug() << "Calculating plot data, plotType:" << static_cast<int>(entries_[index].plotType);
            // In 2D mode, Implicit3D uses GPU rendering (no vertex calculation needed)
            if (!canvas_) {
                qDebug() << "ERROR: canvas_ is null!";
                return;
            }
            bool is3DMode = canvas_->is3DMode();
            qDebug() << "is3DMode:" << is3DMode;
            if (entries_[index].plotType == PlotType::Surface3D ||
                entries_[index].plotType == PlotType::Parametric3D ||
                (entries_[index].plotType == PlotType::Implicit3D && is3DMode)) {
                qDebug() << "Calling calculatePlotData3D";
                calculatePlotData3D(entries_[index]);
            } else if (entries_[index].plotType != PlotType::Implicit3D) {
                qDebug() << "Calling calculatePlotData";
                calculatePlotData(entries_[index]);
            } else {
                qDebug() << "Skipping calculation for Implicit3D in 2D mode";
            }
            // Implicit3D in 2D mode: no calculation needed, GPU shader handles it
            qDebug() << "Plot data calculated";
        }
    } catch (const std::exception& e) {
        entries_[index].hasError = true;
        entries_[index].errorMessage = e.what();
        entries_[index].compiledExpr = nullptr;
    } catch (...) {
        entries_[index].hasError = true;
        entries_[index].errorMessage = "未知错误";
        entries_[index].compiledExpr = nullptr;
    }

    qDebug() << "Updating side panel";
    sidePanel_->updateEntry(index, entries_[index]);
    qDebug() << "Setting plot entries to canvas";
    canvas_->setPlotEntries(entries_);
    qDebug() << "onEntryChanged done";
}

void MainWindow::onEntryDeleted(int index) {
    if (index < 0 || index >= static_cast<int>(entries_.size())) return;

    entries_.erase(entries_.begin() + index);
    sidePanel_->removeEntry(index);
    canvas_->setPlotEntries(entries_);
}

void MainWindow::onViewChanged(QPointF /*offset*/, double /*scale*/) {
    recalculateAll();
}

void MainWindow::recalculateAll() {
    bool is3DMode = canvas_->is3DMode();
    for (auto& entry : entries_) {
        if (!entry.hasError && entry.compiledExpr) {
            // In 2D mode, Implicit3D uses GPU rendering (no vertex calculation needed)
            if (entry.plotType == PlotType::Surface3D ||
                entry.plotType == PlotType::Parametric3D ||
                (entry.plotType == PlotType::Implicit3D && is3DMode)) {
                calculatePlotData3D(entry);
            } else if (entry.plotType != PlotType::Implicit3D) {
                calculatePlotData(entry);
            }
            // Implicit3D in 2D mode: no calculation needed, GPU shader handles it
        }
    }
    canvas_->setPlotEntries(entries_);
}

void MainWindow::parseAndCompileEntry(PlotEntry& entry) {
    entry.hasError = false;
    entry.errorMessage.clear();
    entry.compiledExpr = nullptr;

    if (entry.expression.empty()) {
        return;
    }

    std::string expr = entry.expression;

    // 检测是否是函数定义: name(params) = body
    std::string funcName;
    std::vector<std::string> params;
    std::string bodyStr;

    if (parseFunctionDefinition(expr, funcName, params, bodyStr)) {
        // If function already exists, treat as function call (implicit plot), not redefinition
        if (userFunctions_.count(funcName) == 0) {
            // 验证函数体语法
            auto bodyExpr = parser_->parse(bodyStr);
            if (parser_->hasError() || !bodyExpr) {
                entry.hasError = true;
                entry.errorMessage = parser_->hasError() ? parser_->getError() : "解析失败";
                return;
            }

            // 存储用户函数（只存储字符串，不存储AST）
            UserFunction func;
            func.params = params;
            func.bodyStr = bodyStr;
            userFunctions_[funcName] = func;

            // 函数定义不需要绘图
            entry.plotType = PlotType::ExplicitY;
            return;
        }
        // Check if this is a function definition (params match) vs function call (params differ)
        const auto& existingFunc = userFunctions_.at(funcName);
        if (params == existingFunc.params) {
            // This is a function definition, not a call - don't plot
            entry.plotType = PlotType::ExplicitY;
            return;
        }
        // Fall through to equation parsing for existing function calls like g(x,y) = 1
    }

    // 检测表达式类型
    size_t eqPos = expr.find('=');

    if (eqPos != std::string::npos) {
        std::string lhs = expr.substr(0, eqPos);
        std::string rhs = expr.substr(eqPos + 1);

        // 去除空格
        lhs.erase(std::remove_if(lhs.begin(), lhs.end(), ::isspace), lhs.end());
        rhs.erase(std::remove_if(rhs.begin(), rhs.end(), ::isspace), rhs.end());

        if (lhs == "y") {
            entry.plotType = PlotType::ExplicitY;
            entry.compiledExpr = parser_->parse(rhs);
        } else if (lhs == "x") {
            entry.plotType = PlotType::ExplicitX;
            entry.compiledExpr = parser_->parse(rhs);
        } else if (lhs == "z") {
            // z = f(x,y) -> Surface3D
            entry.plotType = PlotType::Surface3D;
            entry.compiledExpr = parser_->parse(rhs);
        } else {
            // Check if it's an implicit 3D equation (contains x, y, and z)
            std::string implicitExpr = lhs + "-(" + rhs + ")";
            auto tempExpr = parser_->parse(implicitExpr);
            if (tempExpr && containsVariable(tempExpr, "z")) {
                // Implicit 3D: f(x,y,z) = 0
                entry.plotType = PlotType::Implicit3D;
                entry.compiledExpr = tempExpr;
            } else {
                // 隐函数 2D: convert "lhs = rhs" to "lhs - (rhs)" for f(x,y) = 0
                entry.plotType = PlotType::Implicit;
                entry.compiledExpr = tempExpr;
            }
        }
    } else {
        // 默认为 y = f(x)
        entry.plotType = PlotType::ExplicitY;
        entry.compiledExpr = parser_->parse(expr);
    }

    if (parser_->hasError()) {
        entry.hasError = true;
        entry.errorMessage = parser_->getError();
        entry.compiledExpr = nullptr;
    }
}

void MainWindow::calculatePlotData(PlotEntry& entry) {
    entry.vertices.clear();
    entry.plotPoints.clear();

    if (!entry.compiledExpr) return;

    double scale = canvas_->getScale();
    QPointF offset = canvas_->getOffset();
    int width = canvas_->width();
    int height = canvas_->height();

    if (entry.plotType == PlotType::ExplicitY) {
        // y = f(x)
        double xMin = (0 - offset.x()) / scale;
        double xMax = (width - offset.x()) / scale;
        double step = 1.0 / (scale * precisionMultiplier_); // 每像素一个点

        std::vector<double> xValues;
        for (double x = xMin; x <= xMax; x += step) {
            xValues.push_back(x);
        }

        std::vector<double> yValues;
        evaluator_->evaluateBatch(entry.compiledExpr, xValues, yValues, variables_, "x");

        for (size_t i = 0; i < xValues.size(); ++i) {
            if (std::isfinite(yValues[i])) {
                float screenX = static_cast<float>(offset.x() + xValues[i] * scale);
                float screenY = static_cast<float>(offset.y() - yValues[i] * scale);

                // 只添加在视口内的点
                if (screenY >= -1000 && screenY <= height + 1000) {
                    entry.vertices.push_back(screenX);
                    entry.vertices.push_back(screenY);
                    entry.plotPoints.push_back(Point2D(xValues[i], yValues[i]));
                }
            } else if (!entry.vertices.empty()) {
                // 遇到NaN时断开线条，添加一个特殊标记
                entry.vertices.push_back(std::nanf(""));
                entry.vertices.push_back(std::nanf(""));
            }
        }
    }
    else if (entry.plotType == PlotType::ExplicitX) {
        // x = f(y)
        double yMin = (offset.y() - height) / scale;
        double yMax = offset.y() / scale;
        double step = 1.0 / (scale * precisionMultiplier_);

        std::vector<double> yInputs;
        for (double y = yMin; y <= yMax; y += step) {
            yInputs.push_back(y);
        }

        std::vector<double> xValues;
        evaluator_->evaluateBatch(entry.compiledExpr, yInputs, xValues, variables_, "y");

        for (size_t i = 0; i < yInputs.size(); ++i) {
            if (std::isfinite(xValues[i])) {
                float screenX = static_cast<float>(offset.x() + xValues[i] * scale);
                float screenY = static_cast<float>(offset.y() - yInputs[i] * scale);

                if (screenX >= -1000 && screenX <= width + 1000) {
                    entry.vertices.push_back(screenX);
                    entry.vertices.push_back(screenY);
                }
            }
        }
    }
    else if (entry.plotType == PlotType::Implicit) {
        // Marching squares algorithm for implicit functions f(x,y) = 0
        double xMin = (0 - offset.x()) / scale;
        double xMax = (width - offset.x()) / scale;
        double yMin = (offset.y() - height) / scale;
        double yMax = offset.y() / scale;

        int gridSize = std::min(static_cast<int>(std::max(width, height) / 4 * precisionMultiplier_), 1000);
        double dx = (xMax - xMin) / gridSize;
        double dy = (yMax - yMin) / gridSize;

        std::vector<std::vector<double>> grid(gridSize + 1, std::vector<double>(gridSize + 1));

        #pragma omp parallel for collapse(2)
        for (int i = 0; i <= gridSize; ++i) {
            for (int j = 0; j <= gridSize; ++j) {
                VariableContext vars = variables_;
                vars["x"] = xMin + i * dx;
                vars["y"] = yMin + j * dy;
                try {
                    grid[i][j] = evaluator_->evaluate(entry.compiledExpr, vars);
                } catch (...) {
                    grid[i][j] = std::nan("");
                }
            }
        }

        auto lerp = [](double p1, double p2, double v1, double v2) {
            if (std::abs(v2 - v1) < 1e-10) return (p1 + p2) / 2;
            return p1 + (-v1) * (p2 - p1) / (v2 - v1);
        };

        auto addSeg = [&](double ax, double ay, double bx, double by) {
            entry.vertices.push_back(static_cast<float>(offset.x() + ax * scale));
            entry.vertices.push_back(static_cast<float>(offset.y() - ay * scale));
            entry.vertices.push_back(static_cast<float>(offset.x() + bx * scale));
            entry.vertices.push_back(static_cast<float>(offset.y() - by * scale));
            entry.vertices.push_back(std::nanf(""));
            entry.vertices.push_back(std::nanf(""));
        };

        for (int i = 0; i < gridSize; ++i) {
            for (int j = 0; j < gridSize; ++j) {
                // Corners: 0=BL, 1=BR, 2=TR, 3=TL
                double v0 = grid[i][j], v1 = grid[i+1][j];
                double v2 = grid[i+1][j+1], v3 = grid[i][j+1];

                if (!std::isfinite(v0) || !std::isfinite(v1) ||
                    !std::isfinite(v2) || !std::isfinite(v3)) continue;

                double x0 = xMin + i * dx, x1 = xMin + (i+1) * dx;
                double y0 = yMin + j * dy, y1 = yMin + (j+1) * dy;

                // Case index: bit0=v0, bit1=v1, bit2=v2, bit3=v3
                int c = (v0 > 0 ? 1 : 0) | (v1 > 0 ? 2 : 0) |
                        (v2 > 0 ? 4 : 0) | (v3 > 0 ? 8 : 0);

                if (c == 0 || c == 15) continue;

                // Edge crossings: bottom(0-1), right(1-2), top(3-2), left(0-3)
                double bx = lerp(x0, x1, v0, v1), by = y0;
                double rx = x1, ry = lerp(y0, y1, v1, v2);
                double tx = lerp(x0, x1, v3, v2), ty = y1;
                double lx = x0, ly = lerp(y0, y1, v0, v3);

                switch (c) {
                    case 1: case 14: addSeg(bx, by, lx, ly); break;
                    case 2: case 13: addSeg(bx, by, rx, ry); break;
                    case 3: case 12: addSeg(lx, ly, rx, ry); break;
                    case 4: case 11: addSeg(rx, ry, tx, ty); break;
                    case 6: case 9:  addSeg(bx, by, tx, ty); break;
                    case 7: case 8:  addSeg(lx, ly, tx, ty); break;
                    case 5:  addSeg(bx, by, lx, ly); addSeg(rx, ry, tx, ty); break;
                    case 10: addSeg(bx, by, rx, ry); addSeg(lx, ly, tx, ty); break;
                }
            }
        }
    }
}

void MainWindow::onParameterChanged(int index, const QString& name, double value) {
    if (index < 0 || index >= static_cast<int>(entries_.size())) return;

    // Update the parameter value in the entry
    for (auto& param : entries_[index].parameters) {
        if (param.name == name.toUtf8().constData()) {
            param.value = value;
            break;
        }
    }

    // Update global variables
    variables_[name.toUtf8().constData()] = value;

    // Recalculate the plot
    if (!entries_[index].hasError && entries_[index].compiledExpr) {
        calculatePlotData(entries_[index]);
        canvas_->setPlotEntries(entries_);
    }
}

void MainWindow::extractParameters(PlotEntry& entry) {
    if (!entry.compiledExpr) return;

    // Collect all variables from the expression
    std::set<std::string> vars;
    collectVariables(entry.compiledExpr, vars);

    // Standard plot variables to exclude
    static const std::set<std::string> standardVars = {"x", "y", "t", "theta", "r"};

    // Build list of parameters (non-standard variables)
    std::vector<ParameterInfo> newParams;
    for (const auto& var : vars) {
        if (standardVars.find(var) == standardVars.end()) {
            // Check if this parameter already exists (preserve its value)
            bool found = false;
            for (const auto& existing : entry.parameters) {
                if (existing.name == var) {
                    newParams.push_back(existing);
                    found = true;
                    break;
                }
            }
            if (!found) {
                ParameterInfo param;
                param.name = var;
                // Check if there's a global value for this variable
                auto it = variables_.find(var);
                param.value = (it != variables_.end()) ? it->second : 1.0;
                newParams.push_back(param);
            }
            // Ensure the variable is in the global context
            if (variables_.find(var) == variables_.end()) {
                variables_[var] = newParams.back().value;
            }
        }
    }

    entry.parameters = std::move(newParams);
}

void MainWindow::collectVariables(const ExprNodePtr& node, std::set<std::string>& vars) {
    if (!node) return;

    switch (node->type) {
        case NodeType::Variable:
            vars.insert(node->name);
            break;
        case NodeType::BinaryOp:
            collectVariables(node->left, vars);
            collectVariables(node->right, vars);
            break;
        case NodeType::UnaryOp:
            collectVariables(node->left, vars);
            break;
        case NodeType::Function:
            for (const auto& arg : node->args) {
                collectVariables(arg, vars);
            }
            break;
        default:
            break;
    }
}

bool MainWindow::containsVariable(const ExprNodePtr& node, const std::string& varName) {
    if (!node) return false;

    switch (node->type) {
        case NodeType::Variable:
            return node->name == varName;
        case NodeType::BinaryOp:
            return containsVariable(node->left, varName) || containsVariable(node->right, varName);
        case NodeType::UnaryOp:
            return containsVariable(node->left, varName);
        case NodeType::Function:
            for (const auto& arg : node->args) {
                if (containsVariable(arg, varName)) return true;
            }
            return false;
        default:
            return false;
    }
}

void MainWindow::setPrecisionMultiplier(double multiplier) {
    precisionMultiplier_ = std::clamp(multiplier, 0.1, 4.0);
    recalculateAll();
}

void MainWindow::onEntryVisibilityChanged(int index, bool visible) {
    if (index < 0 || index >= static_cast<int>(entries_.size())) return;
    entries_[index].visible = visible;
    canvas_->setPlotEntries(entries_);
}

void MainWindow::onEntryColorChanged(int index, const Color& color) {
    if (index < 0 || index >= static_cast<int>(entries_.size())) return;
    entries_[index].color = color;
    canvas_->setPlotEntries(entries_);
}

void MainWindow::calculatePlotData3D(PlotEntry& entry) {
    qDebug() << "calculatePlotData3D: start";
    entry.vertices3D.clear();
    entry.indices3D.clear();
    entry.plotPoints3D.clear();

    if (!entry.compiledExpr) {
        qDebug() << "calculatePlotData3D: no compiled expr";
        return;
    }

    if (entry.plotType == PlotType::Surface3D) {
        qDebug() << "calculatePlotData3D: Surface3D";
        // z = f(x,y) surface
        QVector3D center = canvas_->getCameraTarget();
        double range = 5.0;
        int resolution = static_cast<int>(50 * precisionMultiplier_);

        std::vector<double> xVals, yVals;
        double step = 2.0 * range / resolution;
        for (int i = 0; i <= resolution; ++i) {
            xVals.push_back(center.x() - range + i * step);
            yVals.push_back(center.y() - range + i * step);
        }

        std::vector<std::vector<double>> zGrid;
        evaluator_->evaluateGrid(entry.compiledExpr, xVals, yVals, zGrid, variables_);

        generateSurfaceMesh(entry, zGrid, xVals, yVals);
    }
    else if (entry.plotType == PlotType::Implicit3D) {
        qDebug() << "calculatePlotData3D: Implicit3D";
        // f(x,y,z) = 0 implicit surface
        QVector3D center = canvas_->getCameraTarget();
        double range = 3.0;
        int resolution = static_cast<int>(30 * precisionMultiplier_);
        qDebug() << "calculatePlotData3D: resolution =" << resolution;

        std::vector<double> xVals, yVals, zVals;
        double step = 2.0 * range / resolution;
        for (int i = 0; i <= resolution; ++i) {
            xVals.push_back(center.x() - range + i * step);
            yVals.push_back(center.y() - range + i * step);
            zVals.push_back(center.z() - range + i * step);
        }
        qDebug() << "calculatePlotData3D: created value arrays";

        std::vector<double> field;
        qDebug() << "calculatePlotData3D: calling evaluateVolume";
        evaluator_->evaluateVolume(entry.compiledExpr, xVals, yVals, zVals, field, variables_);
        qDebug() << "calculatePlotData3D: evaluateVolume done, field size =" << field.size();

        generateImplicit3DMesh(entry, field, resolution + 1, resolution + 1, resolution + 1,
                               center.x() - range, center.x() + range,
                               center.y() - range, center.y() + range,
                               center.z() - range, center.z() + range);
        qDebug() << "calculatePlotData3D: generateImplicit3DMesh done, vertices3D size:" << entry.vertices3D.size();
    }
    else if (entry.plotType == PlotType::Parametric3D) {
        // x=f(t), y=g(t), z=h(t) parametric curve
        if (!entry.compiledExprX || !entry.compiledExprY || !entry.compiledExprZ) return;

        double tMin = 0.0, tMax = 2.0 * M_PI;
        int numPoints = static_cast<int>(500 * precisionMultiplier_);
        double step = (tMax - tMin) / numPoints;

        std::vector<double> tVals;
        for (int i = 0; i <= numPoints; ++i) {
            tVals.push_back(tMin + i * step);
        }

        std::vector<double> xVals, yVals, zVals;
        evaluator_->evaluateBatch(entry.compiledExprX, tVals, xVals, variables_, "t");
        evaluator_->evaluateBatch(entry.compiledExprY, tVals, yVals, variables_, "t");
        evaluator_->evaluateBatch(entry.compiledExprZ, tVals, zVals, variables_, "t");

        for (size_t i = 0; i < tVals.size(); ++i) {
            if (std::isfinite(xVals[i]) && std::isfinite(yVals[i]) && std::isfinite(zVals[i])) {
                entry.vertices3D.push_back(static_cast<float>(xVals[i]));
                entry.vertices3D.push_back(static_cast<float>(yVals[i]));
                entry.vertices3D.push_back(static_cast<float>(zVals[i]));
            }
        }
    }
}

void MainWindow::generateSurfaceMesh(PlotEntry& entry, const std::vector<std::vector<double>>& zGrid,
                                     const std::vector<double>& xVals, const std::vector<double>& yVals) {
    int ny = static_cast<int>(yVals.size());
    int nx = static_cast<int>(xVals.size());

    // Generate triangles with normals
    for (int j = 0; j < ny - 1; ++j) {
        for (int i = 0; i < nx - 1; ++i) {
            double z00 = zGrid[j][i];
            double z10 = zGrid[j][i + 1];
            double z01 = zGrid[j + 1][i];
            double z11 = zGrid[j + 1][i + 1];

            if (!std::isfinite(z00) || !std::isfinite(z10) ||
                !std::isfinite(z01) || !std::isfinite(z11)) continue;

            float x0 = static_cast<float>(xVals[i]);
            float x1 = static_cast<float>(xVals[i + 1]);
            float y0 = static_cast<float>(yVals[j]);
            float y1 = static_cast<float>(yVals[j + 1]);

            // Calculate normals for the two triangles
            auto calcNormal = [](float ax, float ay, float az,
                                 float bx, float by, float bz,
                                 float cx, float cy, float cz) {
                float ux = bx - ax, uy = by - ay, uz = bz - az;
                float vx = cx - ax, vy = cy - ay, vz = cz - az;
                float nx = uy * vz - uz * vy;
                float ny = uz * vx - ux * vz;
                float nz = ux * vy - uy * vx;
                float len = std::sqrt(nx * nx + ny * ny + nz * nz);
                if (len > 0.0001f) { nx /= len; ny /= len; nz /= len; }
                return std::make_tuple(nx, ny, nz);
            };

            // Triangle 1: (x0,y0,z00), (x1,y0,z10), (x0,y1,z01)
            auto [n1x, n1y, n1z] = calcNormal(x0, y0, static_cast<float>(z00),
                                              x1, y0, static_cast<float>(z10),
                                              x0, y1, static_cast<float>(z01));

            // Vertex 1
            entry.vertices3D.push_back(x0);
            entry.vertices3D.push_back(static_cast<float>(z00));
            entry.vertices3D.push_back(y0);
            entry.vertices3D.push_back(n1x);
            entry.vertices3D.push_back(n1z);
            entry.vertices3D.push_back(n1y);

            // Vertex 2
            entry.vertices3D.push_back(x1);
            entry.vertices3D.push_back(static_cast<float>(z10));
            entry.vertices3D.push_back(y0);
            entry.vertices3D.push_back(n1x);
            entry.vertices3D.push_back(n1z);
            entry.vertices3D.push_back(n1y);

            // Vertex 3
            entry.vertices3D.push_back(x0);
            entry.vertices3D.push_back(static_cast<float>(z01));
            entry.vertices3D.push_back(y1);
            entry.vertices3D.push_back(n1x);
            entry.vertices3D.push_back(n1z);
            entry.vertices3D.push_back(n1y);

            // Triangle 2: (x1,y0,z10), (x1,y1,z11), (x0,y1,z01)
            auto [n2x, n2y, n2z] = calcNormal(x1, y0, static_cast<float>(z10),
                                              x1, y1, static_cast<float>(z11),
                                              x0, y1, static_cast<float>(z01));

            // Vertex 4
            entry.vertices3D.push_back(x1);
            entry.vertices3D.push_back(static_cast<float>(z10));
            entry.vertices3D.push_back(y0);
            entry.vertices3D.push_back(n2x);
            entry.vertices3D.push_back(n2z);
            entry.vertices3D.push_back(n2y);

            // Vertex 5
            entry.vertices3D.push_back(x1);
            entry.vertices3D.push_back(static_cast<float>(z11));
            entry.vertices3D.push_back(y1);
            entry.vertices3D.push_back(n2x);
            entry.vertices3D.push_back(n2z);
            entry.vertices3D.push_back(n2y);

            // Vertex 6
            entry.vertices3D.push_back(x0);
            entry.vertices3D.push_back(static_cast<float>(z01));
            entry.vertices3D.push_back(y1);
            entry.vertices3D.push_back(n2x);
            entry.vertices3D.push_back(n2z);
            entry.vertices3D.push_back(n2y);
        }
    }
}

// Marching cubes edge table
static const int mcEdgeTable[256] = {
    0x0, 0x109, 0x203, 0x30a, 0x406, 0x50f, 0x605, 0x70c,
    0x80c, 0x905, 0xa0f, 0xb06, 0xc0a, 0xd03, 0xe09, 0xf00,
    0x190, 0x99, 0x393, 0x29a, 0x596, 0x49f, 0x795, 0x69c,
    0x99c, 0x895, 0xb9f, 0xa96, 0xd9a, 0xc93, 0xf99, 0xe90,
    0x230, 0x339, 0x33, 0x13a, 0x636, 0x73f, 0x435, 0x53c,
    0xa3c, 0xb35, 0x83f, 0x936, 0xe3a, 0xf33, 0xc39, 0xd30,
    0x3a0, 0x2a9, 0x1a3, 0xaa, 0x7a6, 0x6af, 0x5a5, 0x4ac,
    0xbac, 0xaa5, 0x9af, 0x8a6, 0xfaa, 0xea3, 0xda9, 0xca0,
    0x460, 0x569, 0x663, 0x76a, 0x66, 0x16f, 0x265, 0x36c,
    0xc6c, 0xd65, 0xe6f, 0xf66, 0x86a, 0x963, 0xa69, 0xb60,
    0x5f0, 0x4f9, 0x7f3, 0x6fa, 0x1f6, 0xff, 0x3f5, 0x2fc,
    0xdfc, 0xcf5, 0xfff, 0xef6, 0x9fa, 0x8f3, 0xbf9, 0xaf0,
    0x650, 0x759, 0x453, 0x55a, 0x256, 0x35f, 0x55, 0x15c,
    0xe5c, 0xf55, 0xc5f, 0xd56, 0xa5a, 0xb53, 0x859, 0x950,
    0x7c0, 0x6c9, 0x5c3, 0x4ca, 0x3c6, 0x2cf, 0x1c5, 0xcc,
    0xfcc, 0xec5, 0xdcf, 0xcc6, 0xbca, 0xac3, 0x9c9, 0x8c0,
    0x8c0, 0x9c9, 0xac3, 0xbca, 0xcc6, 0xdcf, 0xec5, 0xfcc,
    0xcc, 0x1c5, 0x2cf, 0x3c6, 0x4ca, 0x5c3, 0x6c9, 0x7c0,
    0x950, 0x859, 0xb53, 0xa5a, 0xd56, 0xc5f, 0xf55, 0xe5c,
    0x15c, 0x55, 0x35f, 0x256, 0x55a, 0x453, 0x759, 0x650,
    0xaf0, 0xbf9, 0x8f3, 0x9fa, 0xef6, 0xfff, 0xcf5, 0xdfc,
    0x2fc, 0x3f5, 0xff, 0x1f6, 0x6fa, 0x7f3, 0x4f9, 0x5f0,
    0xb60, 0xa69, 0x963, 0x86a, 0xf66, 0xe6f, 0xd65, 0xc6c,
    0x36c, 0x265, 0x16f, 0x66, 0x76a, 0x663, 0x569, 0x460,
    0xca0, 0xda9, 0xea3, 0xfaa, 0x8a6, 0x9af, 0xaa5, 0xbac,
    0x4ac, 0x5a5, 0x6af, 0x7a6, 0xaa, 0x1a3, 0x2a9, 0x3a0,
    0xd30, 0xc39, 0xf33, 0xe3a, 0x936, 0x83f, 0xb35, 0xa3c,
    0x53c, 0x435, 0x73f, 0x636, 0x13a, 0x33, 0x339, 0x230,
    0xe90, 0xf99, 0xc93, 0xd9a, 0xa96, 0xb9f, 0x895, 0x99c,
    0x69c, 0x795, 0x49f, 0x596, 0x29a, 0x393, 0x99, 0x190,
    0xf00, 0xe09, 0xd03, 0xc0a, 0xb06, 0xa0f, 0x905, 0x80c,
    0x70c, 0x605, 0x50f, 0x406, 0x30a, 0x203, 0x109, 0x0
};

// Marching cubes triangle table (simplified - only first few cases)
static const int mcTriTable[256][16] = {
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 1, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 8, 3, 9, 8, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 2, 10, 0, 2, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {2, 8, 3, 2, 10, 8, 10, 9, 8, -1, -1, -1, -1, -1, -1, -1},
    {3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 11, 2, 8, 11, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 9, 0, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 11, 2, 1, 9, 11, 9, 8, 11, -1, -1, -1, -1, -1, -1, -1},
    {3, 10, 1, 11, 10, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 10, 1, 0, 8, 10, 8, 11, 10, -1, -1, -1, -1, -1, -1, -1},
    {3, 9, 0, 3, 11, 9, 11, 10, 9, -1, -1, -1, -1, -1, -1, -1},
    {9, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 3, 0, 7, 3, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 1, 9, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 1, 9, 4, 7, 1, 7, 3, 1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 4, 7, 3, 0, 4, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1},
    {9, 2, 10, 9, 0, 2, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1},
    {2, 10, 9, 2, 9, 7, 2, 7, 3, 7, 9, 4, -1, -1, -1, -1},
    {8, 4, 7, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {11, 4, 7, 11, 2, 4, 2, 0, 4, -1, -1, -1, -1, -1, -1, -1},
    {9, 0, 1, 8, 4, 7, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1},
    {4, 7, 11, 9, 4, 11, 9, 11, 2, 9, 2, 1, -1, -1, -1, -1},
    {3, 10, 1, 3, 11, 10, 7, 8, 4, -1, -1, -1, -1, -1, -1, -1},
    {1, 11, 10, 1, 4, 11, 1, 0, 4, 7, 11, 4, -1, -1, -1, -1},
    {4, 7, 8, 9, 0, 11, 9, 11, 10, 11, 0, 3, -1, -1, -1, -1},
    {4, 7, 11, 4, 11, 9, 9, 11, 10, -1, -1, -1, -1, -1, -1, -1},
    {9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 5, 4, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 5, 4, 1, 5, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {8, 5, 4, 8, 3, 5, 3, 1, 5, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 0, 8, 1, 2, 10, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1},
    {5, 2, 10, 5, 4, 2, 4, 0, 2, -1, -1, -1, -1, -1, -1, -1},
    {2, 10, 5, 3, 2, 5, 3, 5, 4, 3, 4, 8, -1, -1, -1, -1},
    {9, 5, 4, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 11, 2, 0, 8, 11, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1},
    {0, 5, 4, 0, 1, 5, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1},
    {2, 1, 5, 2, 5, 8, 2, 8, 11, 4, 8, 5, -1, -1, -1, -1},
    {10, 3, 11, 10, 1, 3, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1},
    {4, 9, 5, 0, 8, 1, 8, 10, 1, 8, 11, 10, -1, -1, -1, -1},
    {5, 4, 0, 5, 0, 11, 5, 11, 10, 11, 0, 3, -1, -1, -1, -1},
    {5, 4, 8, 5, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1},
    {9, 7, 8, 5, 7, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 3, 0, 9, 5, 3, 5, 7, 3, -1, -1, -1, -1, -1, -1, -1},
    {0, 7, 8, 0, 1, 7, 1, 5, 7, -1, -1, -1, -1, -1, -1, -1},
    {1, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 7, 8, 9, 5, 7, 10, 1, 2, -1, -1, -1, -1, -1, -1, -1},
    {10, 1, 2, 9, 5, 0, 5, 3, 0, 5, 7, 3, -1, -1, -1, -1},
    {8, 0, 2, 8, 2, 5, 8, 5, 7, 10, 5, 2, -1, -1, -1, -1},
    {2, 10, 5, 2, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1},
    {7, 9, 5, 7, 8, 9, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1},
    {9, 5, 7, 9, 7, 2, 9, 2, 0, 2, 7, 11, -1, -1, -1, -1},
    {2, 3, 11, 0, 1, 8, 1, 7, 8, 1, 5, 7, -1, -1, -1, -1},
    {11, 2, 1, 11, 1, 7, 7, 1, 5, -1, -1, -1, -1, -1, -1, -1},
    {9, 5, 8, 8, 5, 7, 10, 1, 3, 10, 3, 11, -1, -1, -1, -1},
    {5, 7, 0, 5, 0, 9, 7, 11, 0, 1, 0, 10, 11, 10, 0, -1},
    {11, 10, 0, 11, 0, 3, 10, 5, 0, 8, 0, 7, 5, 7, 0, -1},
    {11, 10, 5, 7, 11, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 0, 1, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 8, 3, 1, 9, 8, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1},
    {1, 6, 5, 2, 6, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 6, 5, 1, 2, 6, 3, 0, 8, -1, -1, -1, -1, -1, -1, -1},
    {9, 6, 5, 9, 0, 6, 0, 2, 6, -1, -1, -1, -1, -1, -1, -1},
    {5, 9, 8, 5, 8, 2, 5, 2, 6, 3, 2, 8, -1, -1, -1, -1},
    {2, 3, 11, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {11, 0, 8, 11, 2, 0, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1},
    {0, 1, 9, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1},
    {5, 10, 6, 1, 9, 2, 9, 11, 2, 9, 8, 11, -1, -1, -1, -1},
    {6, 3, 11, 6, 5, 3, 5, 1, 3, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 11, 0, 11, 5, 0, 5, 1, 5, 11, 6, -1, -1, -1, -1},
    {3, 11, 6, 0, 3, 6, 0, 6, 5, 0, 5, 9, -1, -1, -1, -1},
    {6, 5, 9, 6, 9, 11, 11, 9, 8, -1, -1, -1, -1, -1, -1, -1},
    {5, 10, 6, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 3, 0, 4, 7, 3, 6, 5, 10, -1, -1, -1, -1, -1, -1, -1},
    {1, 9, 0, 5, 10, 6, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1},
    {10, 6, 5, 1, 9, 7, 1, 7, 3, 7, 9, 4, -1, -1, -1, -1},
    {6, 1, 2, 6, 5, 1, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 5, 5, 2, 6, 3, 0, 4, 3, 4, 7, -1, -1, -1, -1},
    {8, 4, 7, 9, 0, 5, 0, 6, 5, 0, 2, 6, -1, -1, -1, -1},
    {7, 3, 9, 7, 9, 4, 3, 2, 9, 5, 9, 6, 2, 6, 9, -1},
    {3, 11, 2, 7, 8, 4, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1},
    {5, 10, 6, 4, 7, 2, 4, 2, 0, 2, 7, 11, -1, -1, -1, -1},
    {0, 1, 9, 4, 7, 8, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1},
    {9, 2, 1, 9, 11, 2, 9, 4, 11, 7, 11, 4, 5, 10, 6, -1},
    {8, 4, 7, 3, 11, 5, 3, 5, 1, 5, 11, 6, -1, -1, -1, -1},
    {5, 1, 11, 5, 11, 6, 1, 0, 11, 7, 11, 4, 0, 4, 11, -1},
    {0, 5, 9, 0, 6, 5, 0, 3, 6, 11, 6, 3, 8, 4, 7, -1},
    {6, 5, 9, 6, 9, 11, 4, 7, 9, 7, 11, 9, -1, -1, -1, -1},
    {10, 4, 9, 6, 4, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 10, 6, 4, 9, 10, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1},
    {10, 0, 1, 10, 6, 0, 6, 4, 0, -1, -1, -1, -1, -1, -1, -1},
    {8, 3, 1, 8, 1, 6, 8, 6, 4, 6, 1, 10, -1, -1, -1, -1},
    {1, 4, 9, 1, 2, 4, 2, 6, 4, -1, -1, -1, -1, -1, -1, -1},
    {3, 0, 8, 1, 2, 9, 2, 4, 9, 2, 6, 4, -1, -1, -1, -1},
    {0, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {8, 3, 2, 8, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1},
    {10, 4, 9, 10, 6, 4, 11, 2, 3, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 2, 2, 8, 11, 4, 9, 10, 4, 10, 6, -1, -1, -1, -1},
    {3, 11, 2, 0, 1, 6, 0, 6, 4, 6, 1, 10, -1, -1, -1, -1},
    {6, 4, 1, 6, 1, 10, 4, 8, 1, 2, 1, 11, 8, 11, 1, -1},
    {9, 6, 4, 9, 3, 6, 9, 1, 3, 11, 6, 3, -1, -1, -1, -1},
    {8, 11, 1, 8, 1, 0, 11, 6, 1, 9, 1, 4, 6, 4, 1, -1},
    {3, 11, 6, 3, 6, 0, 0, 6, 4, -1, -1, -1, -1, -1, -1, -1},
    {6, 4, 8, 11, 6, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {7, 10, 6, 7, 8, 10, 8, 9, 10, -1, -1, -1, -1, -1, -1, -1},
    {0, 7, 3, 0, 10, 7, 0, 9, 10, 6, 7, 10, -1, -1, -1, -1},
    {10, 6, 7, 1, 10, 7, 1, 7, 8, 1, 8, 0, -1, -1, -1, -1},
    {10, 6, 7, 10, 7, 1, 1, 7, 3, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 6, 1, 6, 8, 1, 8, 9, 8, 6, 7, -1, -1, -1, -1},
    {2, 6, 9, 2, 9, 1, 6, 7, 9, 0, 9, 3, 7, 3, 9, -1},
    {7, 8, 0, 7, 0, 6, 6, 0, 2, -1, -1, -1, -1, -1, -1, -1},
    {7, 3, 2, 6, 7, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {2, 3, 11, 10, 6, 8, 10, 8, 9, 8, 6, 7, -1, -1, -1, -1},
    {2, 0, 7, 2, 7, 11, 0, 9, 7, 6, 7, 10, 9, 10, 7, -1},
    {1, 8, 0, 1, 7, 8, 1, 10, 7, 6, 7, 10, 2, 3, 11, -1},
    {11, 2, 1, 11, 1, 7, 10, 6, 1, 6, 7, 1, -1, -1, -1, -1},
    {8, 9, 6, 8, 6, 7, 9, 1, 6, 11, 6, 3, 1, 3, 6, -1},
    {0, 9, 1, 11, 6, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {7, 8, 0, 7, 0, 6, 3, 11, 0, 11, 6, 0, -1, -1, -1, -1},
    {7, 11, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 0, 8, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 1, 9, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {8, 1, 9, 8, 3, 1, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1},
    {10, 1, 2, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, 3, 0, 8, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1},
    {2, 9, 0, 2, 10, 9, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1},
    {6, 11, 7, 2, 10, 3, 10, 8, 3, 10, 9, 8, -1, -1, -1, -1},
    {7, 2, 3, 6, 2, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {7, 0, 8, 7, 6, 0, 6, 2, 0, -1, -1, -1, -1, -1, -1, -1},
    {2, 7, 6, 2, 3, 7, 0, 1, 9, -1, -1, -1, -1, -1, -1, -1},
    {1, 6, 2, 1, 8, 6, 1, 9, 8, 8, 7, 6, -1, -1, -1, -1},
    {10, 7, 6, 10, 1, 7, 1, 3, 7, -1, -1, -1, -1, -1, -1, -1},
    {10, 7, 6, 1, 7, 10, 1, 8, 7, 1, 0, 8, -1, -1, -1, -1},
    {0, 3, 7, 0, 7, 10, 0, 10, 9, 6, 10, 7, -1, -1, -1, -1},
    {7, 6, 10, 7, 10, 8, 8, 10, 9, -1, -1, -1, -1, -1, -1, -1},
    {6, 8, 4, 11, 8, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 6, 11, 3, 0, 6, 0, 4, 6, -1, -1, -1, -1, -1, -1, -1},
    {8, 6, 11, 8, 4, 6, 9, 0, 1, -1, -1, -1, -1, -1, -1, -1},
    {9, 4, 6, 9, 6, 3, 9, 3, 1, 11, 3, 6, -1, -1, -1, -1},
    {6, 8, 4, 6, 11, 8, 2, 10, 1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, 3, 0, 11, 0, 6, 11, 0, 4, 6, -1, -1, -1, -1},
    {4, 11, 8, 4, 6, 11, 0, 2, 9, 2, 10, 9, -1, -1, -1, -1},
    {10, 9, 3, 10, 3, 2, 9, 4, 3, 11, 3, 6, 4, 6, 3, -1},
    {8, 2, 3, 8, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1},
    {0, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 9, 0, 2, 3, 4, 2, 4, 6, 4, 3, 8, -1, -1, -1, -1},
    {1, 9, 4, 1, 4, 2, 2, 4, 6, -1, -1, -1, -1, -1, -1, -1},
    {8, 1, 3, 8, 6, 1, 8, 4, 6, 6, 10, 1, -1, -1, -1, -1},
    {10, 1, 0, 10, 0, 6, 6, 0, 4, -1, -1, -1, -1, -1, -1, -1},
    {4, 6, 3, 4, 3, 8, 6, 10, 3, 0, 3, 9, 10, 9, 3, -1},
    {10, 9, 4, 6, 10, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 9, 5, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, 4, 9, 5, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1},
    {5, 0, 1, 5, 4, 0, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1},
    {11, 7, 6, 8, 3, 4, 3, 5, 4, 3, 1, 5, -1, -1, -1, -1},
    {9, 5, 4, 10, 1, 2, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1},
    {6, 11, 7, 1, 2, 10, 0, 8, 3, 4, 9, 5, -1, -1, -1, -1},
    {7, 6, 11, 5, 4, 10, 4, 2, 10, 4, 0, 2, -1, -1, -1, -1},
    {3, 4, 8, 3, 5, 4, 3, 2, 5, 10, 5, 2, 11, 7, 6, -1},
    {7, 2, 3, 7, 6, 2, 5, 4, 9, -1, -1, -1, -1, -1, -1, -1},
    {9, 5, 4, 0, 8, 6, 0, 6, 2, 6, 8, 7, -1, -1, -1, -1},
    {3, 6, 2, 3, 7, 6, 1, 5, 0, 5, 4, 0, -1, -1, -1, -1},
    {6, 2, 8, 6, 8, 7, 2, 1, 8, 4, 8, 5, 1, 5, 8, -1},
    {9, 5, 4, 10, 1, 6, 1, 7, 6, 1, 3, 7, -1, -1, -1, -1},
    {1, 6, 10, 1, 7, 6, 1, 0, 7, 8, 7, 0, 9, 5, 4, -1},
    {4, 0, 10, 4, 10, 5, 0, 3, 10, 6, 10, 7, 3, 7, 10, -1},
    {7, 6, 10, 7, 10, 8, 5, 4, 10, 4, 8, 10, -1, -1, -1, -1},
    {6, 9, 5, 6, 11, 9, 11, 8, 9, -1, -1, -1, -1, -1, -1, -1},
    {3, 6, 11, 0, 6, 3, 0, 5, 6, 0, 9, 5, -1, -1, -1, -1},
    {0, 11, 8, 0, 5, 11, 0, 1, 5, 5, 6, 11, -1, -1, -1, -1},
    {6, 11, 3, 6, 3, 5, 5, 3, 1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, 9, 5, 11, 9, 11, 8, 11, 5, 6, -1, -1, -1, -1},
    {0, 11, 3, 0, 6, 11, 0, 9, 6, 5, 6, 9, 1, 2, 10, -1},
    {11, 8, 5, 11, 5, 6, 8, 0, 5, 10, 5, 2, 0, 2, 5, -1},
    {6, 11, 3, 6, 3, 5, 2, 10, 3, 10, 5, 3, -1, -1, -1, -1},
    {5, 8, 9, 5, 2, 8, 5, 6, 2, 3, 8, 2, -1, -1, -1, -1},
    {9, 5, 6, 9, 6, 0, 0, 6, 2, -1, -1, -1, -1, -1, -1, -1},
    {1, 5, 8, 1, 8, 0, 5, 6, 8, 3, 8, 2, 6, 2, 8, -1},
    {1, 5, 6, 2, 1, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 3, 6, 1, 6, 10, 3, 8, 6, 5, 6, 9, 8, 9, 6, -1},
    {10, 1, 0, 10, 0, 6, 9, 5, 0, 5, 6, 0, -1, -1, -1, -1},
    {0, 3, 8, 5, 6, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {10, 5, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {11, 5, 10, 7, 5, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {11, 5, 10, 11, 7, 5, 8, 3, 0, -1, -1, -1, -1, -1, -1, -1},
    {5, 11, 7, 5, 10, 11, 1, 9, 0, -1, -1, -1, -1, -1, -1, -1},
    {10, 7, 5, 10, 11, 7, 9, 8, 1, 8, 3, 1, -1, -1, -1, -1},
    {11, 1, 2, 11, 7, 1, 7, 5, 1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, 1, 2, 7, 1, 7, 5, 7, 2, 11, -1, -1, -1, -1},
    {9, 7, 5, 9, 2, 7, 9, 0, 2, 2, 11, 7, -1, -1, -1, -1},
    {7, 5, 2, 7, 2, 11, 5, 9, 2, 3, 2, 8, 9, 8, 2, -1},
    {2, 5, 10, 2, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1},
    {8, 2, 0, 8, 5, 2, 8, 7, 5, 10, 2, 5, -1, -1, -1, -1},
    {9, 0, 1, 5, 10, 3, 5, 3, 7, 3, 10, 2, -1, -1, -1, -1},
    {9, 8, 2, 9, 2, 1, 8, 7, 2, 10, 2, 5, 7, 5, 2, -1},
    {1, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 7, 0, 7, 1, 1, 7, 5, -1, -1, -1, -1, -1, -1, -1},
    {9, 0, 3, 9, 3, 5, 5, 3, 7, -1, -1, -1, -1, -1, -1, -1},
    {9, 8, 7, 5, 9, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {5, 8, 4, 5, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1},
    {5, 0, 4, 5, 11, 0, 5, 10, 11, 11, 3, 0, -1, -1, -1, -1},
    {0, 1, 9, 8, 4, 10, 8, 10, 11, 10, 4, 5, -1, -1, -1, -1},
    {10, 11, 4, 10, 4, 5, 11, 3, 4, 9, 4, 1, 3, 1, 4, -1},
    {2, 5, 1, 2, 8, 5, 2, 11, 8, 4, 5, 8, -1, -1, -1, -1},
    {0, 4, 11, 0, 11, 3, 4, 5, 11, 2, 11, 1, 5, 1, 11, -1},
    {0, 2, 5, 0, 5, 9, 2, 11, 5, 4, 5, 8, 11, 8, 5, -1},
    {9, 4, 5, 2, 11, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {2, 5, 10, 3, 5, 2, 3, 4, 5, 3, 8, 4, -1, -1, -1, -1},
    {5, 10, 2, 5, 2, 4, 4, 2, 0, -1, -1, -1, -1, -1, -1, -1},
    {3, 10, 2, 3, 5, 10, 3, 8, 5, 4, 5, 8, 0, 1, 9, -1},
    {5, 10, 2, 5, 2, 4, 1, 9, 2, 9, 4, 2, -1, -1, -1, -1},
    {8, 4, 5, 8, 5, 3, 3, 5, 1, -1, -1, -1, -1, -1, -1, -1},
    {0, 4, 5, 1, 0, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {8, 4, 5, 8, 5, 3, 9, 0, 5, 0, 3, 5, -1, -1, -1, -1},
    {9, 4, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 11, 7, 4, 9, 11, 9, 10, 11, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, 4, 9, 7, 9, 11, 7, 9, 10, 11, -1, -1, -1, -1},
    {1, 10, 11, 1, 11, 4, 1, 4, 0, 7, 4, 11, -1, -1, -1, -1},
    {3, 1, 4, 3, 4, 8, 1, 10, 4, 7, 4, 11, 10, 11, 4, -1},
    {4, 11, 7, 9, 11, 4, 9, 2, 11, 9, 1, 2, -1, -1, -1, -1},
    {9, 7, 4, 9, 11, 7, 9, 1, 11, 2, 11, 1, 0, 8, 3, -1},
    {11, 7, 4, 11, 4, 2, 2, 4, 0, -1, -1, -1, -1, -1, -1, -1},
    {11, 7, 4, 11, 4, 2, 8, 3, 4, 3, 2, 4, -1, -1, -1, -1},
    {2, 9, 10, 2, 7, 9, 2, 3, 7, 7, 4, 9, -1, -1, -1, -1},
    {9, 10, 7, 9, 7, 4, 10, 2, 7, 8, 7, 0, 2, 0, 7, -1},
    {3, 7, 10, 3, 10, 2, 7, 4, 10, 1, 10, 0, 4, 0, 10, -1},
    {1, 10, 2, 8, 7, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 9, 1, 4, 1, 7, 7, 1, 3, -1, -1, -1, -1, -1, -1, -1},
    {4, 9, 1, 4, 1, 7, 0, 8, 1, 8, 7, 1, -1, -1, -1, -1},
    {4, 0, 3, 7, 4, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 8, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 0, 9, 3, 9, 11, 11, 9, 10, -1, -1, -1, -1, -1, -1, -1},
    {0, 1, 10, 0, 10, 8, 8, 10, 11, -1, -1, -1, -1, -1, -1, -1},
    {3, 1, 10, 11, 3, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 11, 1, 11, 9, 9, 11, 8, -1, -1, -1, -1, -1, -1, -1},
    {3, 0, 9, 3, 9, 11, 1, 2, 9, 2, 11, 9, -1, -1, -1, -1},
    {0, 2, 11, 8, 0, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 2, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {2, 3, 8, 2, 8, 10, 10, 8, 9, -1, -1, -1, -1, -1, -1, -1},
    {9, 10, 2, 0, 9, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {2, 3, 8, 2, 8, 10, 0, 1, 8, 1, 10, 8, -1, -1, -1, -1},
    {1, 10, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 3, 8, 9, 1, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 9, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 3, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
};

void MainWindow::generateImplicit3DMesh(PlotEntry& entry, const std::vector<double>& field,
                                        int nx, int ny, int nz,
                                        double xMin, double xMax,
                                        double yMin, double yMax,
                                        double zMin, double zMax) {
    int triangleCount = 0;
    double dx = (xMax - xMin) / (nx - 1);
    double dy = (yMax - yMin) / (ny - 1);
    double dz = (zMax - zMin) / (nz - 1);

    auto getField = [&](int i, int j, int k) -> double {
        if (i < 0 || i >= nx || j < 0 || j >= ny || k < 0 || k >= nz) return 0.0;
        return field[i + j * nx + k * nx * ny];
    };

    auto lerp = [](double p1, double p2, double v1, double v2) -> double {
        if (std::abs(v2 - v1) < 1e-10) return (p1 + p2) / 2.0;
        return p1 + (-v1) * (p2 - p1) / (v2 - v1);
    };

    // Process each cube
    for (int k = 0; k < nz - 1; ++k) {
        for (int j = 0; j < ny - 1; ++j) {
            for (int i = 0; i < nx - 1; ++i) {
                // Get corner values
                double v[8];
                v[0] = getField(i, j, k);
                v[1] = getField(i + 1, j, k);
                v[2] = getField(i + 1, j + 1, k);
                v[3] = getField(i, j + 1, k);
                v[4] = getField(i, j, k + 1);
                v[5] = getField(i + 1, j, k + 1);
                v[6] = getField(i + 1, j + 1, k + 1);
                v[7] = getField(i, j + 1, k + 1);

                // Check for NaN
                bool hasNaN = false;
                for (int c = 0; c < 8; ++c) {
                    if (!std::isfinite(v[c])) { hasNaN = true; break; }
                }
                if (hasNaN) continue;

                // Calculate cube index
                int cubeIndex = 0;
                if (v[0] > 0) cubeIndex |= 1;
                if (v[1] > 0) cubeIndex |= 2;
                if (v[2] > 0) cubeIndex |= 4;
                if (v[3] > 0) cubeIndex |= 8;
                if (v[4] > 0) cubeIndex |= 16;
                if (v[5] > 0) cubeIndex |= 32;
                if (v[6] > 0) cubeIndex |= 64;
                if (v[7] > 0) cubeIndex |= 128;

                if (mcEdgeTable[cubeIndex] == 0) continue;

                // Corner positions
                double x0 = xMin + i * dx, x1 = xMin + (i + 1) * dx;
                double y0 = yMin + j * dy, y1 = yMin + (j + 1) * dy;
                double z0 = zMin + k * dz, z1 = zMin + (k + 1) * dz;

                // Calculate edge vertices
                double vertList[12][3];
                if (mcEdgeTable[cubeIndex] & 1)    { vertList[0][0] = lerp(x0, x1, v[0], v[1]); vertList[0][1] = y0; vertList[0][2] = z0; }
                if (mcEdgeTable[cubeIndex] & 2)    { vertList[1][0] = x1; vertList[1][1] = lerp(y0, y1, v[1], v[2]); vertList[1][2] = z0; }
                if (mcEdgeTable[cubeIndex] & 4)    { vertList[2][0] = lerp(x0, x1, v[3], v[2]); vertList[2][1] = y1; vertList[2][2] = z0; }
                if (mcEdgeTable[cubeIndex] & 8)    { vertList[3][0] = x0; vertList[3][1] = lerp(y0, y1, v[0], v[3]); vertList[3][2] = z0; }
                if (mcEdgeTable[cubeIndex] & 16)   { vertList[4][0] = lerp(x0, x1, v[4], v[5]); vertList[4][1] = y0; vertList[4][2] = z1; }
                if (mcEdgeTable[cubeIndex] & 32)   { vertList[5][0] = x1; vertList[5][1] = lerp(y0, y1, v[5], v[6]); vertList[5][2] = z1; }
                if (mcEdgeTable[cubeIndex] & 64)   { vertList[6][0] = lerp(x0, x1, v[7], v[6]); vertList[6][1] = y1; vertList[6][2] = z1; }
                if (mcEdgeTable[cubeIndex] & 128)  { vertList[7][0] = x0; vertList[7][1] = lerp(y0, y1, v[4], v[7]); vertList[7][2] = z1; }
                if (mcEdgeTable[cubeIndex] & 256)  { vertList[8][0] = x0; vertList[8][1] = y0; vertList[8][2] = lerp(z0, z1, v[0], v[4]); }
                if (mcEdgeTable[cubeIndex] & 512)  { vertList[9][0] = x1; vertList[9][1] = y0; vertList[9][2] = lerp(z0, z1, v[1], v[5]); }
                if (mcEdgeTable[cubeIndex] & 1024) { vertList[10][0] = x1; vertList[10][1] = y1; vertList[10][2] = lerp(z0, z1, v[2], v[6]); }
                if (mcEdgeTable[cubeIndex] & 2048) { vertList[11][0] = x0; vertList[11][1] = y1; vertList[11][2] = lerp(z0, z1, v[3], v[7]); }

                // Generate triangles
                for (int t = 0; mcTriTable[cubeIndex][t] != -1; t += 3) {
                        triangleCount++;
                        int e0 = mcTriTable[cubeIndex][t];
                        int e1 = mcTriTable[cubeIndex][t + 1];
                        int e2 = mcTriTable[cubeIndex][t + 2];

                        // Get vertices directly (no coordinate swap)
                        float ax = static_cast<float>(vertList[e0][0]);
                        float ay = static_cast<float>(vertList[e0][1]);
                        float az = static_cast<float>(vertList[e0][2]);
                        float bx = static_cast<float>(vertList[e1][0]);
                        float by = static_cast<float>(vertList[e1][1]);
                        float bz = static_cast<float>(vertList[e1][2]);
                        float cx = static_cast<float>(vertList[e2][0]);
                        float cy = static_cast<float>(vertList[e2][1]);
                        float cz = static_cast<float>(vertList[e2][2]);

                        // Calculate normal
                        float ux = bx - ax, uy = by - ay, uz = bz - az;
                        float vx = cx - ax, vy = cy - ay, vz = cz - az;
                        float nx = uy * vz - uz * vy;
                        float ny = uz * vx - ux * vz;
                        float nz = ux * vy - uy * vx;
                        float len = std::sqrt(nx * nx + ny * ny + nz * nz);
                        if (len > 0.0001f) { nx /= len; ny /= len; nz /= len; }

                        // Add vertices with normals
                        entry.vertices3D.push_back(ax);
                        entry.vertices3D.push_back(ay);
                        entry.vertices3D.push_back(az);
                        entry.vertices3D.push_back(nx);
                        entry.vertices3D.push_back(ny);
                        entry.vertices3D.push_back(nz);

                        entry.vertices3D.push_back(bx);
                        entry.vertices3D.push_back(by);
                        entry.vertices3D.push_back(bz);
                        entry.vertices3D.push_back(nx);
                        entry.vertices3D.push_back(ny);
                        entry.vertices3D.push_back(nz);

                        entry.vertices3D.push_back(cx);
                        entry.vertices3D.push_back(cy);
                        entry.vertices3D.push_back(cz);
                        entry.vertices3D.push_back(nx);
                        entry.vertices3D.push_back(ny);
                        entry.vertices3D.push_back(nz);
                    }
            }
        }
    }
    qDebug() << "generateImplicit3DMesh: generated" << triangleCount << "triangles";
}

} // namespace ArchMaths
