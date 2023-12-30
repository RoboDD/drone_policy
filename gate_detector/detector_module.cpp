DetectorModule::DetectorModule(const std::string &yaml_path) {
  loadParameters(yaml_path);
  // create inference engine
  // create inference context
}

void DetectorModule::detectGate(
    const std::vector<cv::Mat> &images, const int &rotate_code,
    std::vector<std::vector<detector_lib::gate_measurement_tt<double>>>
        *const detected_gates) {
  // preprocess images on cpu
  preprocess_cpu(buffer_size_input_.first, buffer_size_input_.second,
                 input_images_, rotate_code);

  // execute neural network on GPU
  context_->execute(batch_size_, buffers_gpu_.data());

  // postprocess results
  performPostprocessing(rotate_code, detected_gates);
}

void DetectorModule::preprocess_cpu(int64_t eltCount, DataType dtype,
                                    const std::vector<cv::Mat> &images,
                                    const int &rotate_code) {
  // prepare input image buffers, potentially rotating the input images
  // copy input image to CUDA memory
  CUDA_API_CALL(cudaMemcpy(buffers_gpu_[0], input_buffer_host_, memSize,
                           cudaMemcpyHostToDevice));
}

void DetectorModule::performPostprocessing(
  // do postprocessing for each image separately
  for (unsigned n = 0; n < batch_size_; n++) {
  // search corner channels for peaks
  // search edges based on detected corners (part affinity fields)
  // get gates by assembling the detected edges together
  // append detected gate to message for publishing
  }
}

