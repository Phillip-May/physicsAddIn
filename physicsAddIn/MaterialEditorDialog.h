#ifndef MATERIALEDITORDIALOG_H
#define MATERIALEDITORDIALOG_H

#include <QDialog>
#include <QLineEdit>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QFormLayout>
#include <QLabel>
#include <QColorDialog>
#include <QColor>
#include "MaterialManager.h"

class MaterialEditorDialog : public QDialog
{
    Q_OBJECT

public:
    explicit MaterialEditorDialog(QWidget* parent = nullptr);
    explicit MaterialEditorDialog(const MaterialProperties& material, QWidget* parent = nullptr);
    
    MaterialProperties getMaterialProperties() const;
    void setMaterialProperties(const MaterialProperties& material);

private slots:
    void onSaveClicked();
    void onCancelClicked();
    void onColorButtonClicked();
    void validateInputs();

private:
    void setupUI();
    void connectSignals();
    
    QLineEdit* m_nameEdit;
    QDoubleSpinBox* m_staticFrictionSpin;
    QDoubleSpinBox* m_dynamicFrictionSpin;
    QDoubleSpinBox* m_restitutionSpin;
    QLineEdit* m_descriptionEdit;
    QPushButton* m_colorButton;
    QPushButton* m_saveButton;
    QPushButton* m_cancelButton;
    
    QColor m_materialColor;
};

#endif // MATERIALEDITORDIALOG_H 