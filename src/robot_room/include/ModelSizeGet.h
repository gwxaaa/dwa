#ifndef MODELSIZEGET_H_
#define MODELSIZEGET_H_

#include <string>

class ModelSizeGet {
public:
    ModelSizeGet(const std::string& modelName);

    void GetModelSize();

private:
    std::string modelName_;
};

#endif /* MODELSIZEGET_H_ */
