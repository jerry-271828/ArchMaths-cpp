#pragma once

#include <cstring>
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <QMatrix4x4>
#include <QPointF>
#include <QVector3D>
#include <memory>
#include "math/MathTypes.h"

namespace ArchMaths {

// Compile expression tree to GLSL code
std::string compileToGLSL(const ExprNodePtr& node);

class GLCanvas : public QOpenGLWidget, protected QOpenGLFunctions {
    Q_OBJECT

public:
    explicit GLCanvas(QWidget* parent = nullptr);
    ~GLCanvas() override;

    void setOffset(const QPointF& offset);
    void setScale(double scale);
    QPointF getOffset() const { return offset_; }
    double getScale() const { return scale_; }

    QPointF screenToMath(const QPointF& screen) const;
    QPointF mathToScreen(const QPointF& math) const;

    void setPlotEntries(const std::vector<PlotEntry>& entries);
    void updatePlotData(int index, const std::vector<float>& vertices);
    void requestRedraw();

    // 3D mode
    void set3DMode(bool enabled);
    bool is3DMode() const { return is3DMode_; }
    void set3DPanMode(bool pan);
    bool is3DPanMode() const { return is3DPanMode_; }
    QVector3D getCameraTarget() const { return cameraTarget_; }
    void zoom3D(float factor);

signals:
    void viewChanged(QPointF offset, double scale);
    void mousePositionChanged(QPointF mathPos);
    void view3DChanged();

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;

private:
    void initShaders();
    void drawGrid();
    void drawAxes();
    void drawPlots();
    void drawImplicitGPU(const PlotEntry& entry);
    bool compileImplicitShader(const ExprNodePtr& expr, const Color& color,
                               const std::vector<ParameterInfo>& parameters);
    void drawAxisLabels();

    // 3D rendering methods
    void initShaders3D();
    void updateCamera();
    void draw3DAxes();
    void draw3DAxisLabels();
    void drawSurface3D(const PlotEntry& entry);
    void drawParametric3D(const PlotEntry& entry);

    std::unique_ptr<QOpenGLShaderProgram> lineShader_;
    std::unique_ptr<QOpenGLShaderProgram> line3DShader_;
    std::unique_ptr<QOpenGLShaderProgram> implicitShader_;
    std::unique_ptr<QOpenGLShaderProgram> surface3DShader_;
    QOpenGLBuffer quadVBO_;

    QOpenGLBuffer gridVBO_;
    QOpenGLBuffer axesVBO_;
    QOpenGLVertexArrayObject vao_;
    std::vector<QOpenGLBuffer> plotVBOs_;
    std::vector<QOpenGLBuffer> plot3DVBOs_;
    std::vector<QOpenGLBuffer> plot3DIBOs_;

    QMatrix4x4 projectionMatrix_;

    QPointF offset_{0.0, 0.0};
    double scale_ = 50.0;
    double minScale_ = 1.0;
    double maxScale_ = 10000.0;

    bool isDragging_ = false;
    QPointF lastMousePos_;

    std::vector<PlotEntry> plotEntries_;
    std::string lastImplicitGLSL_;
    Color lastImplicitColor_;

    // 3D state
    bool is3DMode_ = false;
    bool is3DPanMode_ = false;
    QMatrix4x4 viewMatrix_;
    QMatrix4x4 modelMatrix_;
    float cameraDistance_ = 8.0f;
    float cameraYaw_ = 45.0f;
    float cameraPitch_ = 30.0f;
    QVector3D cameraTarget_{0.0f, 0.0f, 0.0f};
};

} // namespace ArchMaths
