#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_ProjectionPatternPreview.h"

#include <memory>

#include <MVImageCameraFactory.hpp>
#include <PDRImageCamera.h>
#include <MVCameraProjector.h>
#include <FrameListener.h>

class ProjectionPatternPreview : public QMainWindow, public CameraLib::FrameListener, public std::enable_shared_from_this<ProjectionPatternPreview>
{
    Q_OBJECT
private:
    Ui::ProjectionPatternPreviewClass ui;
    std::unique_ptr<QLabel> label_rec_frames_;

    std::shared_ptr<CameraLib::PDNImageCamera> camera_;
    std::shared_ptr<CameraLib::MVCameraProjector> projector_;

    uint8_t patterns_count_;
    uint8_t current_pattern_index_;
    bool is_listener_registered_ {false};
    bool is_continues_projecting_ {false};

    std::array<std::unique_ptr<QImage>, 2> q_images_;
    //std::array<std::unique_ptr<cv::Mat>, 2> frame_buffers_;

    std::array<std::mutex, 2> q_image_mutexes_;
    //std::array<std::mutex, 2> frame_buffer_mutexes_;

    std::array<std::vector<cv::Mat>, 2> pattern_images_;

    bool is_capturing_ {false};

    static void saveImage(const std::string& filename);
    void captureButtonClicked() const;
    void sliderValueChanged() const;
    void saveSeqPatterns();
    void openDirAndCalPhase();

    void frameReadyCallback(cv::InputArray image_data) override;

    bool eventFilter(QObject* obj, QEvent* e) override;

    void copyFrame(const std::vector<cv::Mat>& images, int index);
    bool paintImage(int index);
    void updateParametersUi(const std::shared_ptr<CameraLib::PDNImageCamera>& camera) const;

    void nextPatternButtonClicked();
    void prevPatternButtonClicked();
    void continuesProjectionButtonClicked();

protected:
    void showEvent(QShowEvent *event) override;

public:
    ProjectionPatternPreview(QWidget *parent = nullptr);
    ~ProjectionPatternPreview();


};
