# Test High Dynamic Range (HDR) image processing application.

## Purpose
Combine multiple exposure images into a single HDR image with improved dynamic range, applying appropriate color correction and tone mapping in the process.

## Input
The application takes several command line arguments:

1. `-c config_file`: Configuration file (required) containing:
   * Enable flag
   * Raw bit depth
   * Raw pixel color pattern
   * Data pedestal
   * Image dimensions (width, height)
   * Pixels per line
   * Reference image index

2. `-i input_file`: Input image files (minimum 2 required)
  * BMP format images
  * Each image needs an associated exposure/gain file (.txt)
  * First image should have an AWB (Auto White Balance) file (.awbout)

3. `-t tuning_file`: Optional tuning parameters file

## Processing Actions
1. Image Loading
  * Reads BMP images via `read_bmp_16_noshift()`
  * Reads exposure/gain settings for each image
  * Reads AWB parameters from .awbout file for color correction

2. HDR Processing
  * Creates HDR context via `test_rhdr_create()`
  * Processes multiple exposures via `test_rhdr_process()`
  * Combines multiple exposures into single HDR image
  * Applies tone mapping and gamma correction

3. Output Generation
  * Applies gamma curve from `hdr_gamma_temp.txt`
  * Converts processed image to BMP format
  * Writes output using `write_bmp()`

## Output
The application generates:

1. A BMP file containing the merged HDR image
  * Name format: `input1+input2+input3.bmp`
  * 16-bit per channel RGB
  * Same dimensions as input images

2. Optional debug information if enabled through `DEBUG_LEVEL`
