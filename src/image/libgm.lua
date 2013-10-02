-- FFI bindings to GraphicsMagick:
local ffi = require "ffi"
ffi.cdef
[[
  // free
  void free(void *);

  // Magick types:
  typedef void MagickWand;
  typedef void PixelWand;
  typedef int MagickBooleanType;
  typedef int ExceptionType;
  typedef int size_t;
  typedef int ChannelType;

  // Pixel formats:
  typedef enum
  {
    CharPixel,
    ShortPixel,
    IntPixel,
    LongPixel,
    FloatPixel,
    DoublePixel,
  } StorageType;

  // Resizing filters:
  typedef enum
  {
    UndefinedFilter,
    PointFilter,
    BoxFilter,
    TriangleFilter,
    HermiteFilter,
    HanningFilter,
    HammingFilter,
    BlackmanFilter,
    GaussianFilter,
    QuadraticFilter,
    CubicFilter,
    CatromFilter,
    MitchellFilter,
    JincFilter,
    SincFilter,
    SincFastFilter,
    KaiserFilter,
    WelshFilter,
    ParzenFilter,
    BohmanFilter,
    BartlettFilter,
    LagrangeFilter,
    LanczosFilter,
    LanczosSharpFilter,
    Lanczos2Filter,
    Lanczos2SharpFilter,
    RobidouxFilter,
    RobidouxSharpFilter,
    CosineFilter,
    SplineFilter,
    LanczosRadiusFilter,
    SentinelFilter
  } FilterTypes;

  // Channels:
  typedef enum
  {
    UndefinedChannel,
    RedChannel,     /* RGB Red channel */
    CyanChannel,    /* CMYK Cyan channel */
    GreenChannel,   /* RGB Green channel */
    MagentaChannel, /* CMYK Magenta channel */
    BlueChannel,    /* RGB Blue channel */
    YellowChannel,  /* CMYK Yellow channel */
    OpacityChannel, /* Opacity channel */
    BlackChannel,   /* CMYK Black (K) channel */
    MatteChannel,   /* Same as Opacity channel (deprecated) */
    AllChannels,    /* Color channels */
    GrayChannel     /* Color channels represent an intensity. */
  } ChannelType;

  // Color spaces:
  typedef enum
  {
    UndefinedColorspace,
    RGBColorspace,         /* Plain old RGB colorspace */
    GRAYColorspace,        /* Plain old full-range grayscale */
    TransparentColorspace, /* RGB but preserve matte channel during quantize */
    OHTAColorspace,
    XYZColorspace,         /* CIE XYZ */
    YCCColorspace,         /* Kodak PhotoCD PhotoYCC */
    YIQColorspace,
    YPbPrColorspace,
    YUVColorspace,
    CMYKColorspace,        /* Cyan, magenta, yellow, black, alpha */
    sRGBColorspace,        /* Kodak PhotoCD sRGB */
    HSLColorspace,         /* Hue, saturation, luminosity */
    HWBColorspace,         /* Hue, whiteness, blackness */
    LABColorspace,         /* LAB colorspace not supported yet other than via lcms */
    CineonLogRGBColorspace,/* RGB data with Cineon Log scaling, 2.048 density range */
    Rec601LumaColorspace,  /* Luma (Y) according to ITU-R 601 */
    Rec601YCbCrColorspace, /* YCbCr according to ITU-R 601 */
    Rec709LumaColorspace,  /* Luma (Y) according to ITU-R 709 */
    Rec709YCbCrColorspace  /* YCbCr according to ITU-R 709 */
  } ColorspaceType;

  // Image types (needed to save images in Grayscale or with Alpha Channel)
  typedef enum
  {
    UndefinedType,
    BilevelType,
    GrayscaleType,
    GrayscaleMatteType,
    PaletteType, 
    PaletteMatteType,
    TrueColorType,
    TrueColorMatteType,
    ColorSeparationType,
    ColorSeparationMatteType,
    OptimizeType
  } ImageType;

  // Global context:
  void MagickWandGenesis();
  void InitializeMagick();
  
  // Magick Wand:
  MagickWand* NewMagickWand();
  MagickWand* DestroyMagickWand(MagickWand*);

  // Pixel Wand
  PixelWand* NewPixelWand();
  unsigned int DestroyPixelWand( PixelWand *wand );
  unsigned int PixelSetBlack( PixelWand *wand, const double black );  
  unsigned int PixelSetRed(   PixelWand *wand, const double red );
  unsigned int PixelSetGreen( PixelWand *wand, const double green );
  unsigned int PixelSetBlue(  PixelWand *wand, const double blue );

  // Read/Write:
  MagickBooleanType MagickReadImage(MagickWand*, const char*);
  MagickBooleanType MagickReadImageBlob(MagickWand*, const void*, const size_t);
  MagickBooleanType MagickWriteImage(MagickWand*, const char*);
  unsigned char *MagickWriteImageBlob( MagickWand *wand, size_t *length );

  // Quality:
  unsigned int MagickSetCompressionQuality( MagickWand *wand, const unsigned long quality );
 
  //Exception handling:
  const char* MagickGetException(const MagickWand*, ExceptionType*);

  // Dimensions:
  int MagickGetImageWidth(MagickWand*);
  int MagickGetImageHeight(MagickWand*);

  // Depth
  int MagickGetImageDepth(MagickWand*);
  unsigned int MagickSetImageDepth( MagickWand *wand, const unsigned long depth );

  // Resize:
  MagickBooleanType MagickResizeImage(MagickWand*, const size_t, const size_t, const FilterTypes, const double);

  // Rotate
  unsigned int MagickRotateImage(MagickWand *wand, const PixelWand *background, const double degrees );
  // Set size:
  unsigned int MagickSetSize( MagickWand *wand, const unsigned long columns, const unsigned long rows );

  // Image format (JPEG, PNG, ...)
  const char* MagickGetImageFormat(MagickWand* wand);
  MagickBooleanType MagickSetImageFormat(MagickWand* wand, const char* format);

  // Raw data:
  unsigned int MagickGetImagePixels( MagickWand *wand, const long x_offset, const long y_offset,
                                     const unsigned long columns, const unsigned long rows,
                                     const char *map, const StorageType storage,
                                     unsigned char *pixels );
  unsigned int MagickSetImagePixels( MagickWand *wand, const long x_offset, const long y_offset,
                                     const unsigned long columns, const unsigned long rows,
                                     const char *map, const StorageType storage,
                                     unsigned char *pixels );

   // Flip/Flop
   unsigned int MagickFlipImage( MagickWand *wand );
   unsigned int MagickFlopImage( MagickWand *wand );

   // Colorspace:
   ColorspaceType MagickGetImageColorspace( MagickWand *wand );
   unsigned int MagickSetImageColorspace( MagickWand *wand, const ColorspaceType colorspace );

   // Description
   const char *MagickDescribeImage( MagickWand *wand );

   //  MagickEqualizeImage() equalizes the image histogram.
   unsigned int MagickEqualizeImage(MagickWand *wand);

  // ImageType get and set. For saving Gray scale and Alpha channels
  ImageType MagickGetImageType(MagickWand *);
  ImageType MagickGetImageSavedType(MagickWand *);

  unsigned int MagickSetImageType(MagickWand *,const ImageType);
  unsigned int MagickSetImageSavedType(MagickWand *,const ImageType);
  
]]

-- Load lib:
local clib = util.ffi.load('libGraphicsMagickWand')
-- Initialize lib:
clib.InitializeMagick();

return clib
