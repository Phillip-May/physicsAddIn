#ifndef OBJECTPROPERTIESDIALOG_H
#define OBJECTPROPERTIESDIALOG_H

#include <QDialog>
#include <QLineEdit>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QFormLayout>
#include <QLabel>
#include <QCheckBox>
#include <QComboBox>
#include "CadNode.h"

#include "MaterialManager.h"

class ObjectPropertiesDialog : public QDialog
{
    Q_OBJECT

public:
    explicit ObjectPropertiesDialog(CadNode* item, MaterialManager* materialManager = nullptr, QWidget* parent = nullptr);
    
private slots:
    void onSaveClicked();
    void onCancelClicked();
    void onUseCustomPropertiesToggled(bool checked);
    void onUseCustomCenterOfMassToggled(bool checked);
    void validateInputs();

private:
    void setupUI();
    void connectSignals();
    void updateUI();
    
    CadNode* m_item;
    MaterialManager* m_materialManager;
    
    // UI elements
    QCheckBox* m_useCustomPropertiesCheck;
    QCheckBox* m_useCustomCenterOfMassCheck;
    QDoubleSpinBox* m_massSpin;
    QDoubleSpinBox* m_centerOfMassXSpin;
    QDoubleSpinBox* m_centerOfMassYSpin;
    QDoubleSpinBox* m_centerOfMassZSpin;
    QComboBox* m_materialCombo;
    QPushButton* m_saveButton;
    QPushButton* m_cancelButton;
    
};

#endif // OBJECTPROPERTIESDIALOG_H 
