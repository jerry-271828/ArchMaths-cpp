#include "ui/EntryWidget.h"
#include "ui/ParameterSlider.h"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QColorDialog>

namespace ArchMaths {

EntryWidget::EntryWidget(int index, const PlotEntry& entry, QWidget* parent)
    : QWidget(parent)
    , index_(index)
    , entry_(entry)
{
    setupUI();
    setEntry(entry);
}

void EntryWidget::setupUI() {
    mainLayout_ = new QVBoxLayout(this);
    mainLayout_->setContentsMargins(5, 5, 5, 5);
    mainLayout_->setSpacing(3);

    // First row: color button + expression input
    QHBoxLayout* topRow = new QHBoxLayout();

    colorButton_ = new QPushButton(this);
    colorButton_->setFixedSize(24, 24);
    colorButton_->setCursor(Qt::PointingHandCursor);
    connect(colorButton_, &QPushButton::clicked, this, &EntryWidget::onColorClicked);

    expressionEdit_ = new QLineEdit(this);
    expressionEdit_->setPlaceholderText("输入表达式，如 y=sin(x)");
    connect(expressionEdit_, &QLineEdit::editingFinished, this, &EntryWidget::onExpressionEdited);

    topRow->addWidget(colorButton_);
    topRow->addWidget(expressionEdit_);

    // Second row: visibility + delete button
    QHBoxLayout* bottomRow = new QHBoxLayout();

    visibilityButton_ = new QPushButton("👁", this);
    visibilityButton_->setFixedSize(30, 24);
    visibilityButton_->setCheckable(true);
    visibilityButton_->setChecked(true);
    connect(visibilityButton_, &QPushButton::clicked, this, &EntryWidget::onVisibilityToggled);

    deleteButton_ = new QPushButton("×", this);
    deleteButton_->setFixedSize(30, 24);
    deleteButton_->setStyleSheet("QPushButton { color: red; font-weight: bold; }");
    connect(deleteButton_, &QPushButton::clicked, this, &EntryWidget::onDeleteClicked);

    errorLabel_ = new QLabel(this);
    errorLabel_->setStyleSheet("QLabel { color: red; font-size: 11px; }");
    errorLabel_->setWordWrap(true);
    errorLabel_->hide();

    bottomRow->addWidget(visibilityButton_);
    bottomRow->addStretch();
    bottomRow->addWidget(deleteButton_);

    // Parameter sliders container
    slidersContainer_ = new QWidget(this);
    slidersLayout_ = new QVBoxLayout(slidersContainer_);
    slidersLayout_->setContentsMargins(0, 0, 0, 0);
    slidersLayout_->setSpacing(2);
    slidersContainer_->hide();

    mainLayout_->addLayout(topRow);
    mainLayout_->addLayout(bottomRow);
    mainLayout_->addWidget(slidersContainer_);
    mainLayout_->addWidget(errorLabel_);

    // Style
    setStyleSheet(R"(
        EntryWidget {
            background-color: #f5f5f5;
            border: 1px solid #ddd;
            border-radius: 5px;
        }
    )");
}

void EntryWidget::setEntry(const PlotEntry& entry) {
    entry_ = entry;

    expressionEdit_->setText(QString::fromStdString(entry.expression));
    visibilityButton_->setChecked(entry.visible);
    updateColorButton();

    if (entry.hasError) {
        errorLabel_->setText(QString::fromStdString(entry.errorMessage));
        errorLabel_->show();
        expressionEdit_->setStyleSheet("QLineEdit { border: 1px solid red; }");
    } else {
        errorLabel_->hide();
        expressionEdit_->setStyleSheet("");
    }

    // Update parameter sliders
    updateParameterSliders(entry.parameters);
}

void EntryWidget::updateParameterSliders(const std::vector<ParameterInfo>& parameters) {
    // Clear existing sliders
    for (auto* slider : parameterSliders_) {
        slidersLayout_->removeWidget(slider);
        slider->deleteLater();
    }
    parameterSliders_.clear();

    // Create new sliders for each parameter
    for (const auto& param : parameters) {
        auto* slider = new ParameterSlider(
            QString::fromStdString(param.name),
            param.value,
            param.minValue,
            param.maxValue,
            slidersContainer_
        );
        connect(slider, &ParameterSlider::valueChanged,
                this, &EntryWidget::onParameterValueChanged);
        parameterSliders_.push_back(slider);
        slidersLayout_->addWidget(slider);
    }

    // Show/hide container based on whether there are parameters
    slidersContainer_->setVisible(!parameters.empty());
}

void EntryWidget::updateColorButton() {
    float r, g, b;
    entry_.color.toRGB(r, g, b);

    QString style = QString("QPushButton { background-color: rgb(%1, %2, %3); border: 1px solid #999; border-radius: 3px; }")
        .arg(static_cast<int>(r * 255))
        .arg(static_cast<int>(g * 255))
        .arg(static_cast<int>(b * 255));

    colorButton_->setStyleSheet(style);
}

void EntryWidget::onExpressionEdited() {
    QString text = expressionEdit_->text();
    if (std::string(text.toUtf8().constData()) != entry_.expression) {
        emit expressionChanged(index_, text);
    }
}

void EntryWidget::onDeleteClicked() {
    emit deleteRequested(index_);
}

void EntryWidget::onVisibilityToggled() {
    emit visibilityChanged(index_, visibilityButton_->isChecked());
}

void EntryWidget::onColorClicked() {
    float r, g, b;
    entry_.color.toRGB(r, g, b);

    QColor currentColor(static_cast<int>(r * 255),
                        static_cast<int>(g * 255),
                        static_cast<int>(b * 255));

    QColor newColor = QColorDialog::getColor(currentColor, this, "选择颜色");

    if (newColor.isValid()) {
        // Convert RGB to HSB
        int h, s, v;
        newColor.getHsv(&h, &s, &v);

        entry_.color.h = static_cast<float>(h);
        entry_.color.s = static_cast<float>(s) / 255.0f * 100.0f;
        entry_.color.b = static_cast<float>(v) / 255.0f * 100.0f;

        updateColorButton();
        emit colorClicked(index_, entry_.color);
    }
}

void EntryWidget::onParameterValueChanged(const QString& name, double value) {
    emit parameterChanged(index_, name, value);
}

} // namespace ArchMaths
