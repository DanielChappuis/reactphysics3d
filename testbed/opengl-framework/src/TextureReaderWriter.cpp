/********************************************************************************
* OpenGL-Framework                                                              *
* Copyright (c) 2013 Daniel Chappuis                                            *
*********************************************************************************
*                                                                               *
* This software is provided 'as-is', without any express or implied warranty.   *
* In no event will the authors be held liable for any damages arising from the  *
* use of this software.                                                         *
*                                                                               *
* Permission is granted to anyone to use this software for any purpose,         *
* including commercial applications, and to alter it and redistribute it        *
* freely, subject to the following restrictions:                                *
*                                                                               *
* 1. The origin of this software must not be misrepresented; you must not claim *
*    that you wrote the original software. If you use this software in a        *
*    product, an acknowledgment in the product documentation would be           *
*    appreciated but is not required.                                           *
*                                                                               *
* 2. Altered source versions must be plainly marked as such, and must not be    *
*    misrepresented as being the original software.                             *
*                                                                               *
* 3. This notice may not be removed or altered from any source distribution.    *
*                                                                               *
********************************************************************************/

// Librairies
#include "TextureReaderWriter.h"
#include <string>
#ifdef USE_JPEG_TEXTURE
    #include <jpeglib.h>
    #include <jerror.h>
#endif

using namespace openglframework;
using namespace std;

// Constants
const uint TextureReaderWriter::JPEG_COMPRESSION_QUALITY = 98;

// TGA file Header
#pragma pack(push, 1)
typedef struct {
    unsigned char  identsize;           // size of ID field that follows 18 byte header (0 usually)
    unsigned char  colourmaptype;       // type of colour map 0=none, 1=has palette
    unsigned char  imagetype;           // type of image 0=none,1=indexed,2=rgb,3=grey,+8=rle packed

    short colourmapstart;               // first colour map entry in palette
    short colourmaplength;              // number of colours in palette
    unsigned char  colourmapbits;       // number of bits per palette entry 15,16,24,32

    short xstart;                       // image x origin
    short ystart;                       // image y origin
    short width;                        // image width in pixels
    short height;                       // image height in pixels
    unsigned char  bits;                // image bits per pixel 8,16,24,32
    unsigned char  descriptor;          // image descriptor bits (vh flip bits)

    // pixel data follows header
} TGA_HEADER;
#pragma pack(pop)

// Load a texture from a file
void TextureReaderWriter::loadTextureFromFile(const std::string& filename,
                                              Texture2D& textureToCreate) {

    // Get the extension of the file
    uint startPosExtension = filename.find_last_of(".");
    string extension = filename.substr(startPosExtension+1);

    // Load the file using the correct method
    if (extension == "tga") {
        readTGAPicture(filename, textureToCreate);
    }
#ifdef USE_JPEG_TEXTURE
    else if (extension == "jpg" || extension == "jpeg"){
        readJPEGPicture(filename, textureToCreate);
    }
#endif
    else {

        // Display an error message and throw an exception
        string errorMessage("Error : the TextureLoader class cannot load a file with the extension .");
        errorMessage += extension;
        std::cerr << errorMessage << std::endl;
        throw std::invalid_argument(errorMessage.c_str());
    }
}

// Write a texture to a file
void TextureReaderWriter::writeTextureToFile(const std::string& filename,const Texture2D& texture) {

    // Get the extension of the file
    uint startPosExtension = filename.find_last_of(".");
    string extension = filename.substr(startPosExtension+1);

    // Write the file using the correct method
    if (extension == "tga") {
        writeTGAPicture(filename, texture);
    }
#ifdef USE_JPEG_TEXTURE
    else if (extension == "jpg" || extension == "jpeg"){
        writeJPEGPicture(filename, texture);
    }
#endif
    else {

        // Display an error message and throw an exception
        string errorMessage("Error : the TextureReaderWriter class cannot write a file with the extension .");
        errorMessage += extension;
        std::cerr << errorMessage << std::endl;
        throw std::invalid_argument(errorMessage.c_str());
    }
}

// Load a TGA picture
void TextureReaderWriter::readTGAPicture(const std::string &filename, Texture2D& textureToCreate) {

    // Open the file
    std::ifstream stream(filename.c_str(), std::ios::binary);

    // If we cannot open the file
    if(!stream.is_open()) {

        // Throw an exception and display an error message
        string errorMessage("Error : Cannot open the file " + filename);
        std::cerr << errorMessage << std::endl;
        throw std::runtime_error(errorMessage);
    }

    TGA_HEADER header;
    stream.read((char *)(&header), sizeof(TGA_HEADER));
    assert(header.width <= 4096 && header.width <= 4096 &&
           header.imagetype == 2 && header.bits == 24);

    // Read the file
    uint width = header.width;
    uint height = header.width;
    uint sizeImg = width*height;
    char* data = new char[sizeImg*3];
    assert(data);
    stream.read(data, sizeImg*3);
    for(uint i = 0; i < sizeImg; i++) {
        unsigned pos = i*3;
        unsigned char red = data[pos];
        data[pos] = data[pos + 2];
        data[pos + 2] = red;
    }

    // Close the stream
    stream.close();

    // Create the OpenGL texture using the picture data from the file
    textureToCreate.create(width, height, GL_RGB, GL_RGB, GL_UNSIGNED_BYTE, data);

    // Free the data memory
    delete[] data;
}


// Write a TGA picture
void TextureReaderWriter::writeTGAPicture(const std::string& filename, const Texture2D& texture) {
    assert(texture.getID() != 0);

    // Bind the corresponding texture
    glBindTexture(GL_TEXTURE_2D, texture.getID());

    uint sizeImg = texture.getWidth() * texture.getHeight();

    // Get the bytes form the OpenGL texture
    char* data = new char[sizeImg * 3];
    glGetTexImage(GL_TEXTURE_2D, 0, GL_RGB, GL_UNSIGNED_BYTE, data);

    // Open the file
    std::ofstream stream(filename.c_str(), std::ios::binary);

    // If the file cannot be opened
    if(!stream.is_open()) {

        // Throw an exception and display an error message
        string errorMessage("Error : Cannot create/access the file " + filename);
        std::cerr << errorMessage << std::endl;
        delete[] data;
        throw std::runtime_error(errorMessage);
    }

    // Fill in the TGA header
    TGA_HEADER header;
    header.identsize = 0;           // size of ID field that follows 18 byte header (0 usually)
    header.colourmaptype = 0;       // type of colour map 0=none, 1=has palette
    header.imagetype = 2;           // type of image 0=none,1=indexed,2=rgb,3=grey,+8=rle packed
    header.colourmapstart = 0;                  // first colour map entry in palette
    header.colourmaplength = 0;                 // number of colours in palette
    header.colourmapbits=0;                     // number of bits per palette entry 15,16,24,32
    header.xstart = 0;                          // image x origin
    header.ystart = 0;                          // image y origin
    header.width = (short)texture.getWidth();   // image width in pixels
    header.height = (short)texture.getHeight(); // image height in pixels
    header.bits = 24;                           // image bits per pixel 8,16,24,32
    header.descriptor = 0;                      // image descriptor bits (vh flip bits)

    // Write the header to the file
    stream.write((char*)(&header), sizeof(TGA_HEADER));

    // Write the bytes to the file
    for(uint i = 0; i < sizeImg; i++) {
        unsigned pos = i*3;
        unsigned char red = data[pos];
        data[pos] = data[pos + 2];
        data[pos + 2] = red;
    }
    stream.write(data, sizeImg*3);

    // Close the file
    stream.close();

    // Delete the data
    delete[] data;

    // Unbind the corresponding texture
    glBindTexture(GL_TEXTURE_2D, 0);
}

#ifdef USE_JPEG_TEXTURE

// Read a JPEG picture
void TextureReaderWriter::readJPEGPicture(const std::string& filename, Texture2D& textureToCreate) {

    struct jpeg_decompress_struct info;
    struct jpeg_error_mgr error;

    info.err = jpeg_std_error(&error);
    jpeg_create_decompress(&info);

    // Open the file
    FILE* file = fopen(filename.c_str(), "rb");

    // If we cannot open the file
    if (!file) {

        // Throw an exception and display an error message
        string errorMessage("Error : Cannot open the file " + filename);
        std::cerr << errorMessage << std::endl;
        throw std::runtime_error(errorMessage);
    }

    jpeg_stdio_src(&info, file);
    jpeg_read_header(&info, true);
    jpeg_start_decompress(&info);

    unsigned long x = info.output_width;
    unsigned long y = info.output_height;
    int channels = info.num_components;
    assert(channels == 3);

    unsigned long size = x * y * 3;

    BYTE* data = new BYTE[size];

    BYTE* p1 = data;
    BYTE** p2 = &p1;
    int numlines = 0;

    while(info.output_scanline < info.output_height) {
        numlines = jpeg_read_scanlines(&info, p2, 1);
        *p2 += numlines * 3 * info.output_width;
    }

    jpeg_finish_decompress(&info);   //finish decompressing this file

    // Close the file
    fclose(file);

    // Create the OpenGL texture
    textureToCreate.create(x, y, GL_RGB, GL_RGB, GL_UNSIGNED_BYTE, data);

    // Free allocated memory
    delete[] data;
}

// Write a JPEG picture
void TextureReaderWriter::writeJPEGPicture(const std::string& filename, const Texture2D& texture) {

    struct jpeg_compress_struct info;
    struct jpeg_error_mgr error;

    info.err = jpeg_std_error(&error);
    jpeg_create_compress(&info);

    // Open the file
    FILE* file = fopen(filename.c_str(), "wb");

    if (!file) {
        // Throw an exception and display an error message
        string errorMessage("Error : Cannot write JPEG picture into the file " + filename);
        std::cerr << errorMessage << std::endl;
        throw std::runtime_error(errorMessage);
    }

    // Get the bytes form the OpenGL texture
    char* data = new char[texture.getWidth() * texture.getHeight() * 3];
    glGetTexImage(GL_TEXTURE_2D, 0, GL_RGB, GL_UNSIGNED_BYTE, data);

    jpeg_stdio_dest(&info, file);

    info.image_width = texture.getWidth();
    info.image_height = texture.getHeight();
    info.input_components = 3;
    info.in_color_space = JCS_RGB;
    jpeg_set_defaults(&info);
    jpeg_set_quality(&info, JPEG_COMPRESSION_QUALITY, true);

    jpeg_start_compress(&info, true);

    // Write the data into the file
    JSAMPROW rowPointer;
    int rowStride = texture.getWidth() * 3;
    while (info.next_scanline < info.image_height) {
        rowPointer = (JSAMPROW) &data[info.next_scanline * rowStride];
        jpeg_write_scanlines(&info, &rowPointer, 1);
    }

    jpeg_finish_compress(&info);
    jpeg_destroy_compress(&info);

    // Close the file
    fclose(file);

    // Free allocated memory
    delete[] data;
}

#endif
