# Image segmentation

We can not use image segmentation directly on the R channel of the RGB image, as RGB values are highly sensitive to illumination. So even if the ball is of red color there might be some region where R values will be quite low due to shadows, and hence we convert the RGB image to HSV( Hue Saturation Value).

## References

1. https://www.learnopencv.com/invisibility-cloak-using-color-detection-and-segmentation-with-opencv/

