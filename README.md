# GUI-system-for-infrared-small-target-detection

Introduction

To promote the development of infrared small target detection technology, we have designed a human-computer interaction system based on a set of model driven algorithms and data driven algorithms, aiming to provide a benchmark platform for researchers to learn, compare, and jointly maintain and update.

The relevant code will be made public later.

In addition, we have also compiled a set of evaluation metrics libraries suitable for algorithms in this field, named BinarySOSMetrics. 

The relevant code is published on https://github.com/BinarySOS/BinarySOSMetrics.

The main features of BinarySOSMetrics include:

High Efficiency: Multi-threading.

Device Friendly: All metrics support automatic batch accumulation.

Unified API: All metrics provide the same API, Metric.update(labels, preds) complete the accumulation of batches， Metric.get() get metrics。

Unified Computational: We use the same calculation logic and algorithms for the same type of metrics, ensuring consistency between results.

Supports multiple data formats: Supports multiple input data formats, hwc/chw/bchw/bhwc/image path, more details in ./notebook/tutorial.ipynb


