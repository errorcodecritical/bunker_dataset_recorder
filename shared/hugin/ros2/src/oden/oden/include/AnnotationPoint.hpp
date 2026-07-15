// Copyright (c) Sensrad 2023-2024
#pragma once

#ifndef POINT_H_
#define POINT_H_

#include <cmath>
#include <cstdint>
#include <vector>

// Type definition enabling propagation of annotated data (which sometimes is
// present in bags
class AnnotationPoint {
private:
  int16_t annotation_cluster_idx;
  uint8_t annotation_class;

public:
  AnnotationPoint(const int16_t annotation_cluster_idx,
                  const uint8_t annotation_class)
      : annotation_cluster_idx{annotation_cluster_idx},
        annotation_class{annotation_class} {}

  int16_t getAnnotationClusterIdx() const { return annotation_cluster_idx; }

  uint8_t getAnnotationClass() const { return annotation_class; }
};
typedef std::vector<AnnotationPoint> AnnotationPointCloud;

#endif // POINT_H_H
