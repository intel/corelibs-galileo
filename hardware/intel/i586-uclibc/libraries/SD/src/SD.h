/*
SD.h library class implementation to access SD card 
Copyright (C) 2014 Intel Corporation

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef __SD_H__
#define __SD_H__

#include <Arduino.h>
#include <string>
#include <dirent.h>

#define FILE_READ 0x1
#define FILE_WRITE 0x13

class File : public Stream{
private:
  FILE * _file;
  DIR * _dirp;
  std::string _name;
  std::string _basename;
  boolean check_exists();
public:
  // Enable this to temporarily make the fields "const" (and find bugs)
  //File& operator=(const File& other) {};
  File(FILE *f, char *name);
  File(DIR *d, char *name);
  File(void);
  ~File(void);
  virtual size_t write(uint8_t);
  virtual size_t write(const uint8_t *buf, size_t size);
  virtual int read();
  virtual int peek();
  virtual int available();
  virtual void flush();
  int read(void *buf, uint16_t nbyte);
  boolean seek(uint32_t pos);
  uint32_t position();
  uint32_t size();
  void close();
  operator bool();
  const char * name();
  boolean isDirectory(void);
  File openNextFile(uint8_t mode = FILE_READ);
  void rewindDirectory(void);
};

class SDClass{
private:

public:
  boolean begin(uint8_t csPin = 0);
  File open(const char *filepath, uint8_t mode = FILE_READ);
  boolean exists(char *filepath);
  boolean mkdir(char *filepath);
  boolean remove(char *filepath);
  boolean rmdir(char *filepath);
};

extern SDClass SD;

#endif
