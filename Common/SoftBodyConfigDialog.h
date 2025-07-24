#ifndef SOFTBODYCONFIGDIALOG_H
#define SOFTBODYCONFIGDIALOG_H

#include <QDialog>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QComboBox>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QLabel>
#include <QGroupBox>
#include "IPhysicsEngine.h"

class SoftBodyConfigDialog : public QDialog
{
    Q_OBJECT

public:
    explicit SoftBodyConfigDialog(QWidget* parent = nullptr);
    ~SoftBodyConfigDialog() = default;

    // Get/set configuration
    SoftBodyConfig getConfig() const;
    void setConfig(const SoftBodyConfig& config);

signals:
    void configAccepted(const SoftBodyConfig& config);

private slots:
    void onAccepted();

private:
    void setupUI();

    // UI Components
    // Material group
    QGroupBox* m_materialGroup;
    QDoubleSpinBox* m_densitySpinBox;
    QDoubleSpinBox* m_youngsModulusSpinBox;
    QDoubleSpinBox* m_poissonsRatioSpinBox;
    
    // Simulation group
    QGroupBox* m_simulationGroup;
    QDoubleSpinBox* m_dampingSpinBox;
    QDoubleSpinBox* m_frictionSpinBox;
    
    // Buttons
    QPushButton* m_okButton;
    QPushButton* m_cancelButton;
    QPushButton* m_applyButton;
};

#endif // SOFTBODYCONFIGDIALOG_H 