#include "rendering/GLCanvas.h"
#include <QMouseEvent>
#include <QWheelEvent>
#include <QPainter>
#include <cmath>
#include <sstream>
#include <iostream>

namespace ArchMaths {

// Compile expression tree to GLSL code
std::string compileToGLSL(const ExprNodePtr& node) {
    if (!node) return "0.0";

    switch (node->type) {
        case NodeType::Number: {
            std::ostringstream oss;
            oss << std::fixed << node->value;
            return oss.str();
        }
        case NodeType::Variable:
            return node->name;
        case NodeType::BinaryOp: {
            std::string l = compileToGLSL(node->left);
            std::string r = compileToGLSL(node->right);
            if (node->op == "^") {
                // GLSL pow() is undefined for negative base - use x*x for ^2
                if (node->right && node->right->type == NodeType::Number && node->right->value == 2.0) {
                    return "(" + l + "*" + l + ")";
                }
                return "pow(abs(" + l + ")," + r + ")";
            }
            return "(" + l + node->op + r + ")";
        }
        case NodeType::UnaryOp:
            return "(" + node->op + compileToGLSL(node->left) + ")";
        case NodeType::Function: {
            std::string args;
            for (size_t i = 0; i < node->args.size(); ++i) {
                if (i > 0) args += ",";
                args += compileToGLSL(node->args[i]);
            }
            std::string fn = node->name;
            if (fn == "ln") fn = "log";
            return fn + "(" + args + ")";
        }
        default:
            return "0.0";
    }
}

GLCanvas::GLCanvas(QWidget* parent)
    : QOpenGLWidget(parent)
    , gridVBO_(QOpenGLBuffer::VertexBuffer)
    , axesVBO_(QOpenGLBuffer::VertexBuffer)
    , quadVBO_(QOpenGLBuffer::VertexBuffer)
{
    setMouseTracking(true);
    setFocusPolicy(Qt::StrongFocus);
}

GLCanvas::~GLCanvas() {
    makeCurrent();
    gridVBO_.destroy();
    axesVBO_.destroy();
    quadVBO_.destroy();
    for (auto& vbo : plotVBOs_) {
        vbo.destroy();
    }
    for (auto& vbo : plot3DVBOs_) {
        vbo.destroy();
    }
    for (auto& ibo : plot3DIBOs_) {
        ibo.destroy();
    }
    vao_.destroy();
    doneCurrent();
}

void GLCanvas::initializeGL() {
    initializeOpenGLFunctions();

    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    vao_.create();
    vao_.bind();

    initShaders();
    initShaders3D();

    gridVBO_.create();
    axesVBO_.create();
    quadVBO_.create();

    // Full-screen quad for implicit rendering
    float quadVertices[] = {-1, -1, 1, -1, -1, 1, 1, 1};
    quadVBO_.bind();
    quadVBO_.allocate(quadVertices, sizeof(quadVertices));
    quadVBO_.release();

    vao_.release();
}

void GLCanvas::initShaders() {
    lineShader_ = std::make_unique<QOpenGLShaderProgram>();

#ifdef WASM_BUILD
    // WebGL 2.0 / OpenGL ES 3.0 shaders
    const char* vertexShaderSource = R"(#version 300 es
precision mediump float;
in vec2 aPos;
uniform mat4 projection;
void main() {
    gl_Position = projection * vec4(aPos, 0.0, 1.0);
}
)";

    const char* fragmentShaderSource = R"(#version 300 es
precision mediump float;
uniform vec4 color;
out vec4 FragColor;
void main() {
    FragColor = color;
}
)";
#else
    // OpenGL ES 2.0 / OpenGL 2.1 compatible shaders
    const char* vertexShaderSource = R"(
#ifdef GL_ES
precision mediump float;
#endif
attribute vec2 aPos;
uniform mat4 projection;
void main() {
    gl_Position = projection * vec4(aPos, 0.0, 1.0);
}
)";

    const char* fragmentShaderSource = R"(
#ifdef GL_ES
precision mediump float;
#endif
uniform vec4 color;
void main() {
    gl_FragColor = color;
}
)";
#endif

    lineShader_->addShaderFromSourceCode(QOpenGLShader::Vertex, vertexShaderSource);
    lineShader_->addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentShaderSource);
    lineShader_->bindAttributeLocation("aPos", 0);
    lineShader_->link();
}

void GLCanvas::initShaders3D() {
    surface3DShader_ = std::make_unique<QOpenGLShaderProgram>();

#ifdef WASM_BUILD
    const char* vertSrc = R"(#version 300 es
precision highp float;
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
uniform mat4 mvp;
uniform mat4 model;
out vec3 vNormal;
out vec3 vPos;
void main() {
    vPos = vec3(model * vec4(aPos, 1.0));
    vNormal = mat3(transpose(inverse(model))) * aNormal;
    gl_Position = mvp * vec4(aPos, 1.0);
}
)";

    const char* fragSrc = R"(#version 300 es
precision highp float;
uniform vec4 color;
uniform vec3 lightDir;
in vec3 vNormal;
in vec3 vPos;
out vec4 FragColor;
void main() {
    vec3 norm = normalize(vNormal);
    float diff = abs(dot(norm, normalize(lightDir)));
    float ambient = 0.3;
    float lighting = ambient + (1.0 - ambient) * diff;
    FragColor = vec4(color.rgb * lighting, color.a);
}
)";
#else
    const char* vertSrc = R"(
#ifdef GL_ES
precision highp float;
#endif
attribute vec3 aPos;
attribute vec3 aNormal;
uniform mat4 mvp;
uniform mat4 model;
varying vec3 vNormal;
varying vec3 vPos;
void main() {
    vPos = vec3(model * vec4(aPos, 1.0));
    vNormal = mat3(model) * aNormal;
    gl_Position = mvp * vec4(aPos, 1.0);
}
)";

    const char* fragSrc = R"(
#ifdef GL_ES
precision highp float;
#endif
uniform vec4 color;
uniform vec3 lightDir;
varying vec3 vNormal;
varying vec3 vPos;
void main() {
    vec3 norm = normalize(vNormal);
    float diff = abs(dot(norm, normalize(lightDir)));
    float ambient = 0.3;
    float lighting = ambient + (1.0 - ambient) * diff;
    gl_FragColor = vec4(color.rgb * lighting, color.a);
}
)";
#endif

    surface3DShader_->addShaderFromSourceCode(QOpenGLShader::Vertex, vertSrc);
    surface3DShader_->addShaderFromSourceCode(QOpenGLShader::Fragment, fragSrc);
    surface3DShader_->bindAttributeLocation("aPos", 0);
    surface3DShader_->bindAttributeLocation("aNormal", 1);
    surface3DShader_->link();
}

void GLCanvas::resizeGL(int w, int h) {
    glViewport(0, 0, w, h);
    projectionMatrix_.setToIdentity();
    projectionMatrix_.ortho(0.0f, static_cast<float>(w),
                            static_cast<float>(h), 0.0f,
                            -1.0f, 1.0f);
}

void GLCanvas::paintGL() {
    std::cerr << "paintGL called" << std::endl;

    if (is3DMode_) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glEnable(GL_DEPTH_TEST);

        vao_.bind();
        draw3DAxes();

        for (const auto& entry : plotEntries_) {
            if (!entry.visible) continue;
            if (entry.plotType == PlotType::Surface3D || entry.plotType == PlotType::Implicit3D) {
                drawSurface3D(entry);
            } else if (entry.plotType == PlotType::Parametric3D) {
                drawParametric3D(entry);
            }
        }

        vao_.release();
        glDisable(GL_DEPTH_TEST);
    } else {
        glClear(GL_COLOR_BUFFER_BIT);

        vao_.bind();
        lineShader_->bind();
        lineShader_->setUniformValue("projection", projectionMatrix_);

        drawGrid();
        drawAxes();
        std::cerr << "Drawing plots" << std::endl;
        drawPlots();
        std::cerr << "Plots drawn" << std::endl;

        lineShader_->release();
        vao_.release();

        // Draw axis labels using QPainter (after OpenGL)
        drawAxisLabels();
    }
}

void GLCanvas::drawGrid() {
    float w = static_cast<float>(width());
    float h = static_cast<float>(height());

    double gridStep = 1.0;
    double scaledStep = gridStep * scale_;

    while (scaledStep < 50) { gridStep *= 2; scaledStep = gridStep * scale_; }
    while (scaledStep > 200) { gridStep /= 2; scaledStep = gridStep * scale_; }

    std::vector<float> gridVertices;

    double startX = std::floor((0 - offset_.x()) / scale_ / gridStep) * gridStep;
    double endX = std::ceil((w - offset_.x()) / scale_ / gridStep) * gridStep;

    for (double x = startX; x <= endX; x += gridStep) {
        float screenX = static_cast<float>(offset_.x() + x * scale_);
        gridVertices.push_back(screenX);
        gridVertices.push_back(0);
        gridVertices.push_back(screenX);
        gridVertices.push_back(h);
    }

    double startY = std::floor((offset_.y() - h) / scale_ / gridStep) * gridStep;
    double endY = std::ceil(offset_.y() / scale_ / gridStep) * gridStep;

    for (double y = startY; y <= endY; y += gridStep) {
        float screenY = static_cast<float>(offset_.y() - y * scale_);
        gridVertices.push_back(0);
        gridVertices.push_back(screenY);
        gridVertices.push_back(w);
        gridVertices.push_back(screenY);
    }

    if (!gridVertices.empty()) {
        gridVBO_.bind();
        gridVBO_.allocate(gridVertices.data(), static_cast<int>(gridVertices.size() * sizeof(float)));

        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), nullptr);

        lineShader_->setUniformValue("color", QVector4D(0.9f, 0.9f, 0.9f, 1.0f));
        glLineWidth(1.0f);
        glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(gridVertices.size() / 2));

        gridVBO_.release();
    }
}

void GLCanvas::drawAxes() {
    float w = static_cast<float>(width());
    float h = static_cast<float>(height());

    std::vector<float> axesVertices;

    float yAxis = static_cast<float>(offset_.y());
    if (yAxis >= 0 && yAxis <= h) {
        axesVertices.push_back(0);
        axesVertices.push_back(yAxis);
        axesVertices.push_back(w);
        axesVertices.push_back(yAxis);
    }

    float xAxis = static_cast<float>(offset_.x());
    if (xAxis >= 0 && xAxis <= w) {
        axesVertices.push_back(xAxis);
        axesVertices.push_back(0);
        axesVertices.push_back(xAxis);
        axesVertices.push_back(h);
    }

    if (!axesVertices.empty()) {
        axesVBO_.bind();
        axesVBO_.allocate(axesVertices.data(), static_cast<int>(axesVertices.size() * sizeof(float)));

        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), nullptr);

        lineShader_->setUniformValue("color", QVector4D(0.3f, 0.3f, 0.3f, 1.0f));
        glLineWidth(2.0f);
        glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(axesVertices.size() / 2));

        axesVBO_.release();
    }
}

void GLCanvas::drawPlots() {
    for (size_t i = 0; i < plotEntries_.size(); ++i) {
        const auto& entry = plotEntries_[i];
        if (!entry.visible) continue;

        // Use GPU rendering for implicit functions
        // In 2D mode, Implicit3D is rendered as a 2D slice with z as a parameter
        if (entry.plotType == PlotType::Implicit || entry.plotType == PlotType::Implicit3D) {
            drawImplicitGPU(entry);
            continue;
        }

        if (entry.vertices.empty()) continue;

        // Ensure we have enough VBOs
        while (plotVBOs_.size() <= i) {
            plotVBOs_.emplace_back(QOpenGLBuffer::VertexBuffer);
            plotVBOs_.back().create();
        }

        // Filter out NaN values and split into segments
        std::vector<float> cleanVertices;
        std::vector<std::pair<int, int>> segments;
        int segmentStart = 0;
        int segmentCount = 0;

        for (size_t j = 0; j + 1 < entry.vertices.size(); j += 2) {
            float x = entry.vertices[j];
            float y = entry.vertices[j + 1];

            if (std::isfinite(x) && std::isfinite(y)) {
                cleanVertices.push_back(x);
                cleanVertices.push_back(y);
                segmentCount++;
            } else {
                if (segmentCount > 1) {
                    segments.emplace_back(segmentStart, segmentCount);
                }
                segmentStart = static_cast<int>(cleanVertices.size() / 2);
                segmentCount = 0;
            }
        }
        if (segmentCount > 1) {
            segments.emplace_back(segmentStart, segmentCount);
        }

        if (cleanVertices.empty()) continue;

        auto& vbo = plotVBOs_[i];
        vbo.bind();
        vbo.allocate(cleanVertices.data(), static_cast<int>(cleanVertices.size() * sizeof(float)));

        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), nullptr);

        float r, g, b;
        entry.color.toRGB(r, g, b);
        lineShader_->setUniformValue("color", QVector4D(r, g, b, 1.0f));
        glLineWidth(entry.thickness);

        for (const auto& seg : segments) {
            glDrawArrays(GL_LINE_STRIP, seg.first, seg.second);
        }

        vbo.release();
    }
}

void GLCanvas::setOffset(const QPointF& offset) {
    offset_ = offset;
    update();
    emit viewChanged(offset_, scale_);
}

void GLCanvas::setScale(double scale) {
    scale_ = std::clamp(scale, minScale_, maxScale_);
    update();
    emit viewChanged(offset_, scale_);
}

QPointF GLCanvas::screenToMath(const QPointF& screen) const {
    return QPointF(
        (screen.x() - offset_.x()) / scale_,
        (offset_.y() - screen.y()) / scale_
    );
}

QPointF GLCanvas::mathToScreen(const QPointF& math) const {
    return QPointF(
        offset_.x() + math.x() * scale_,
        offset_.y() - math.y() * scale_
    );
}

void GLCanvas::setPlotEntries(const std::vector<PlotEntry>& entries) {
    plotEntries_ = entries;
    update();
}

void GLCanvas::updatePlotData(int index, const std::vector<float>& vertices) {
    if (index >= 0 && index < static_cast<int>(plotEntries_.size())) {
        plotEntries_[index].vertices = vertices;
        update();
    }
}

void GLCanvas::requestRedraw() {
    update();
}

void GLCanvas::mousePressEvent(QMouseEvent* event) {
    if (event->button() == Qt::LeftButton) {
        isDragging_ = true;
        lastMousePos_ = event->pos();
        setCursor(Qt::ClosedHandCursor);
    }
}

void GLCanvas::mouseMoveEvent(QMouseEvent* event) {
    if (is3DMode_) {
        if (isDragging_) {
            QPointF delta = event->pos() - lastMousePos_;
            cameraYaw_ += static_cast<float>(delta.x()) * 0.5f;
            cameraPitch_ = std::clamp(cameraPitch_ - static_cast<float>(delta.y()) * 0.5f, -89.0f, 89.0f);
            lastMousePos_ = event->pos();
            updateCamera();
            update();
        }
    } else {
        QPointF mathPos = screenToMath(event->pos());
        emit mousePositionChanged(mathPos);

        if (isDragging_) {
            QPointF delta = event->pos() - lastMousePos_;
            offset_ += delta;
            lastMousePos_ = event->pos();
            update();
            emit viewChanged(offset_, scale_);
        }
    }
}

void GLCanvas::mouseReleaseEvent(QMouseEvent* event) {
    if (event->button() == Qt::LeftButton) {
        isDragging_ = false;
        setCursor(Qt::ArrowCursor);
    }
}

void GLCanvas::wheelEvent(QWheelEvent* event) {
    if (is3DMode_) {
        float factor = event->angleDelta().y() > 0 ? 0.9f : 1.1f;
        cameraDistance_ = std::clamp(cameraDistance_ * factor, 1.0f, 50.0f);
        updateCamera();
        update();
    } else {
        QPointF mousePos = event->position();
        QPointF mathPosBefore = screenToMath(mousePos);

        double factor = event->angleDelta().y() > 0 ? 1.1 : 0.9;
        double newScale = std::clamp(scale_ * factor, minScale_, maxScale_);

        if (newScale != scale_) {
            scale_ = newScale;
            offset_.setX(mousePos.x() - mathPosBefore.x() * scale_);
            offset_.setY(mousePos.y() + mathPosBefore.y() * scale_);
            update();
            emit viewChanged(offset_, scale_);
        }
    }
}

bool GLCanvas::compileImplicitShader(const ExprNodePtr& expr, const Color& color,
                                     const std::vector<ParameterInfo>& parameters) {
    std::string glslExpr = compileToGLSL(expr);
    bool sameColor = (color.h == lastImplicitColor_.h && color.s == lastImplicitColor_.s &&
                      color.b == lastImplicitColor_.b && color.a == lastImplicitColor_.a);
    if (glslExpr == lastImplicitGLSL_ && sameColor && implicitShader_) return true;
    lastImplicitGLSL_ = glslExpr;
    lastImplicitColor_ = color;

    float r, g, b;
    color.toRGB(r, g, b);

    // Format floats with decimal point for valid GLSL
    auto formatFloat = [](float f) {
        std::ostringstream oss;
        oss << std::fixed << f;
        return oss.str();
    };
    std::string colorVec = "vec4(" + formatFloat(r) + "," + formatFloat(g) + "," + formatFloat(b) + ", alpha)";

    // Build uniform declarations for parameters
    std::string paramUniforms;
    for (const auto& param : parameters) {
        paramUniforms += "uniform float " + param.name + ";\n";
    }

#ifdef WASM_BUILD
    // WebGL 2.0 / OpenGL ES 3.0 shaders
    std::string vertSrc = R"(#version 300 es
precision highp float;
in vec2 aPos;
out vec2 vPos;
void main() {
    vPos = aPos;
    gl_Position = vec4(aPos, 0.0, 1.0);
}
)";

    std::string fragSrc = R"(#version 300 es
precision highp float;
in vec2 vPos;
out vec4 FragColor;
uniform vec2 offset;
uniform float scale;
uniform vec2 resolution;
)" + paramUniforms + R"(
float f(float x, float y) {
    return )" + glslExpr + R"(;
}

void main() {
    vec2 screen = (vPos * 0.5 + 0.5) * resolution;
    float x = (screen.x - offset.x) / scale;
    float y = (offset.y - resolution.y + screen.y) / scale;

    float v = f(x, y);
    float dx = dFdx(v);
    float dy = dFdy(v);
    float grad = sqrt(dx*dx + dy*dy);
    float d = abs(v) / max(grad, 0.0001);

    float alpha = smoothstep(2.0, 0.5, d);
    if (alpha < 0.01) discard;
    FragColor = )" + colorVec + R"(;
}
)";
#else
    // OpenGL ES 2.0 compatible shaders
    std::string vertSrc = R"(
#ifdef GL_ES
precision highp float;
#endif
attribute vec2 aPos;
varying vec2 vPos;
void main() {
    vPos = aPos;
    gl_Position = vec4(aPos, 0.0, 1.0);
}
)";

    std::string fragSrc = R"(
#ifdef GL_ES
precision highp float;
#endif
varying vec2 vPos;
uniform vec2 offset;
uniform float scale;
uniform vec2 resolution;
)" + paramUniforms + R"(
float f(float x, float y) {
    return )" + glslExpr + R"(;
}

void main() {
    vec2 screen = (vPos * 0.5 + 0.5) * resolution;
    float x = (screen.x - offset.x) / scale;
    float y = (offset.y - resolution.y + screen.y) / scale;

    float v = f(x, y);
    float dx = dFdx(v);
    float dy = dFdy(v);
    float grad = sqrt(dx*dx + dy*dy);
    float d = abs(v) / max(grad, 0.0001);

    float alpha = smoothstep(2.0, 0.5, d);
    if (alpha < 0.01) discard;
    gl_FragColor = )" + colorVec + R"(;
}
)";
#endif

    implicitShader_ = std::make_unique<QOpenGLShaderProgram>();
    if (!implicitShader_->addShaderFromSourceCode(QOpenGLShader::Vertex, vertSrc.c_str())) return false;
    if (!implicitShader_->addShaderFromSourceCode(QOpenGLShader::Fragment, fragSrc.c_str())) return false;
    implicitShader_->bindAttributeLocation("aPos", 0);
    return implicitShader_->link();
}

void GLCanvas::drawImplicitGPU(const PlotEntry& entry) {
    if (!entry.compiledExpr || !compileImplicitShader(entry.compiledExpr, entry.color, entry.parameters)) return;

    implicitShader_->bind();
    implicitShader_->setUniformValue("offset", QVector2D(offset_.x(), offset_.y()));
    implicitShader_->setUniformValue("scale", static_cast<float>(scale_));
    implicitShader_->setUniformValue("resolution", QVector2D(width(), height()));

    // Set parameter uniforms
    for (const auto& param : entry.parameters) {
        implicitShader_->setUniformValue(param.name.c_str(), static_cast<float>(param.value));
    }

    quadVBO_.bind();
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, nullptr);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    glDisableVertexAttribArray(0);
    quadVBO_.release();
    implicitShader_->release();
    lineShader_->bind();
}

void GLCanvas::drawAxisLabels() {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setPen(QColor(80, 80, 80));
    painter.setFont(QFont("Sans", 9));

    float w = static_cast<float>(width());
    float h = static_cast<float>(height());

    // Calculate grid step (same logic as drawGrid)
    double gridStep = 1.0;
    double scaledStep = gridStep * scale_;
    while (scaledStep < 50) { gridStep *= 2; scaledStep = gridStep * scale_; }
    while (scaledStep > 200) { gridStep /= 2; scaledStep = gridStep * scale_; }

    // X-axis labels
    float yAxis = static_cast<float>(offset_.y());
    if (yAxis >= 0 && yAxis <= h) {
        double startX = std::floor((0 - offset_.x()) / scale_ / gridStep) * gridStep;
        double endX = std::ceil((w - offset_.x()) / scale_ / gridStep) * gridStep;

        for (double x = startX; x <= endX; x += gridStep) {
            if (std::abs(x) < gridStep * 0.01) continue; // Skip origin
            float screenX = static_cast<float>(offset_.x() + x * scale_);

            // Format label
            QString label;
            if (std::abs(x - std::round(x)) < 0.0001)
                label = QString::number(static_cast<int>(std::round(x)));
            else
                label = QString::number(x, 'g', 4);

            // Draw tick mark
            painter.drawLine(QPointF(screenX, yAxis - 3), QPointF(screenX, yAxis + 3));
            // Draw label below x-axis
            QRectF rect(screenX - 20, yAxis + 5, 40, 15);
            painter.drawText(rect, Qt::AlignCenter, label);
        }
    }

    // Y-axis labels
    float xAxis = static_cast<float>(offset_.x());
    if (xAxis >= 0 && xAxis <= w) {
        double startY = std::floor((offset_.y() - h) / scale_ / gridStep) * gridStep;
        double endY = std::ceil(offset_.y() / scale_ / gridStep) * gridStep;

        for (double y = startY; y <= endY; y += gridStep) {
            if (std::abs(y) < gridStep * 0.01) continue; // Skip origin
            float screenY = static_cast<float>(offset_.y() - y * scale_);

            // Format label
            QString label;
            if (std::abs(y - std::round(y)) < 0.0001)
                label = QString::number(static_cast<int>(std::round(y)));
            else
                label = QString::number(y, 'g', 4);

            // Draw tick mark
            painter.drawLine(QPointF(xAxis - 3, screenY), QPointF(xAxis + 3, screenY));
            // Draw label to the left of y-axis
            QRectF rect(xAxis - 35, screenY - 8, 30, 16);
            painter.drawText(rect, Qt::AlignRight | Qt::AlignVCenter, label);
        }
    }

    painter.end();
}

void GLCanvas::set3DMode(bool enabled) {
    is3DMode_ = enabled;
    if (enabled) {
        updateCamera();
    }
    update();
}

void GLCanvas::updateCamera() {
    float yawRad = static_cast<float>(cameraYaw_ * M_PI / 180.0);
    float pitchRad = static_cast<float>(cameraPitch_ * M_PI / 180.0);

    QVector3D cameraPos(
        cameraDistance_ * std::cos(pitchRad) * std::sin(yawRad),
        cameraDistance_ * std::sin(pitchRad),
        cameraDistance_ * std::cos(pitchRad) * std::cos(yawRad)
    );

    viewMatrix_.setToIdentity();
    viewMatrix_.lookAt(cameraPos + cameraTarget_, cameraTarget_, QVector3D(0, 1, 0));

    modelMatrix_.setToIdentity();
}

void GLCanvas::draw3DAxes() {
    // Set up 3D projection
    QMatrix4x4 projection;
    float aspect = static_cast<float>(width()) / static_cast<float>(height());
    projection.perspective(45.0f, aspect, 0.1f, 100.0f);

    QMatrix4x4 mvp = projection * viewMatrix_ * modelMatrix_;

    lineShader_->bind();
    lineShader_->setUniformValue("projection", mvp);

    // Draw 3D axes
    std::vector<float> axisVertices = {
        // X axis (red)
        -5, 0, 0,  5, 0, 0,
        // Y axis (green)
        0, -5, 0,  0, 5, 0,
        // Z axis (blue)
        0, 0, -5,  0, 0, 5
    };

    axesVBO_.bind();
    axesVBO_.allocate(axisVertices.data(), static_cast<int>(axisVertices.size() * sizeof(float)));

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);

    glLineWidth(2.0f);

    // X axis - red
    lineShader_->setUniformValue("color", QVector4D(0.8f, 0.2f, 0.2f, 1.0f));
    glDrawArrays(GL_LINES, 0, 2);

    // Y axis - green
    lineShader_->setUniformValue("color", QVector4D(0.2f, 0.8f, 0.2f, 1.0f));
    glDrawArrays(GL_LINES, 2, 2);

    // Z axis - blue
    lineShader_->setUniformValue("color", QVector4D(0.2f, 0.2f, 0.8f, 1.0f));
    glDrawArrays(GL_LINES, 4, 2);

    axesVBO_.release();
    lineShader_->release();
}

void GLCanvas::drawSurface3D(const PlotEntry& entry) {
    if (entry.vertices3D.empty()) return;

    // Ensure we have enough VBOs
    size_t idx = &entry - plotEntries_.data();
    while (plot3DVBOs_.size() <= idx) {
        plot3DVBOs_.emplace_back(QOpenGLBuffer::VertexBuffer);
        plot3DVBOs_.back().create();
    }
    while (plot3DIBOs_.size() <= idx) {
        plot3DIBOs_.emplace_back(QOpenGLBuffer::IndexBuffer);
        plot3DIBOs_.back().create();
    }

    QMatrix4x4 projection;
    float aspect = static_cast<float>(width()) / static_cast<float>(height());
    projection.perspective(45.0f, aspect, 0.1f, 100.0f);

    QMatrix4x4 mvp = projection * viewMatrix_ * modelMatrix_;

    surface3DShader_->bind();
    surface3DShader_->setUniformValue("mvp", mvp);
    surface3DShader_->setUniformValue("model", modelMatrix_);
    surface3DShader_->setUniformValue("lightDir", QVector3D(1.0f, 1.0f, 1.0f));

    float r, g, b;
    entry.color.toRGB(r, g, b);
    surface3DShader_->setUniformValue("color", QVector4D(r, g, b, 0.9f));

    auto& vbo = plot3DVBOs_[idx];
    vbo.bind();
    vbo.allocate(entry.vertices3D.data(), static_cast<int>(entry.vertices3D.size() * sizeof(float)));

    // Position attribute (location 0)
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);

    // Normal attribute (location 1)
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), reinterpret_cast<void*>(3 * sizeof(float)));

    if (!entry.indices3D.empty()) {
        auto& ibo = plot3DIBOs_[idx];
        ibo.bind();
        ibo.allocate(entry.indices3D.data(), static_cast<int>(entry.indices3D.size() * sizeof(unsigned int)));
        glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(entry.indices3D.size()), GL_UNSIGNED_INT, nullptr);
        ibo.release();
    } else {
        // Draw as triangles directly (vertices3D contains pos+normal interleaved)
        glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(entry.vertices3D.size() / 6));
    }

    glDisableVertexAttribArray(0);
    glDisableVertexAttribArray(1);
    vbo.release();
    surface3DShader_->release();
}

void GLCanvas::drawParametric3D(const PlotEntry& entry) {
    if (entry.vertices3D.empty()) return;

    size_t idx = &entry - plotEntries_.data();
    while (plot3DVBOs_.size() <= idx) {
        plot3DVBOs_.emplace_back(QOpenGLBuffer::VertexBuffer);
        plot3DVBOs_.back().create();
    }

    QMatrix4x4 projection;
    float aspect = static_cast<float>(width()) / static_cast<float>(height());
    projection.perspective(45.0f, aspect, 0.1f, 100.0f);

    QMatrix4x4 mvp = projection * viewMatrix_ * modelMatrix_;

    lineShader_->bind();
    lineShader_->setUniformValue("projection", mvp);

    float r, g, b;
    entry.color.toRGB(r, g, b);
    lineShader_->setUniformValue("color", QVector4D(r, g, b, 1.0f));

    auto& vbo = plot3DVBOs_[idx];
    vbo.bind();
    vbo.allocate(entry.vertices3D.data(), static_cast<int>(entry.vertices3D.size() * sizeof(float)));

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);

    glLineWidth(entry.thickness);
    glDrawArrays(GL_LINE_STRIP, 0, static_cast<GLsizei>(entry.vertices3D.size() / 3));

    glDisableVertexAttribArray(0);
    vbo.release();
    lineShader_->release();
}

} // namespace ArchMaths
