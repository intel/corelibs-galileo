/*
SD.cpp library class implementation to access SD card 
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

#include "SD.h"
#include "trace.h"

#include <string>

#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <mntent.h>
#include <string.h>
#include <libgen.h>

#include <sys/types.h>
#include <sys/stat.h>

#define MY_TRACE_PREFIX "SD"

const char* SD_MOUNT_PATH = "/media/mmcblk0p1";

// Check if the SD card is mounted by searching through the mtab
static boolean check_sd(){
  FILE * mtab = NULL;
  struct mntent * part = NULL;
  if((mtab = setmntent("/etc/mtab", "r")) != NULL){
    while((part = getmntent(mtab)) != NULL){
      if((part->mnt_dir != NULL) && (strcmp(part->mnt_dir, SD_MOUNT_PATH)) == 0){
        return(true);
      }
    }
    endmntent(mtab);
  }
  trace_error("Unable to locate SD mount path: %s\n", SD_MOUNT_PATH);
  return(false);
}

std::string sd_path(const char *filepath){
  return std::string(SD_MOUNT_PATH) + "/" + filepath;
}

std::string cpp_basename(const std::string fullname)
{
  char work[256];
  const int sofw = sizeof(work);
  // FIXME: trace_error if strlen(fullname) > sofw
  snprintf(work, sofw, "%s", fullname.c_str());
  return basename(work);
}

File::File(FILE *f_arg, char *n_arg) :
  _file(f_arg), _dirp(NULL), _name(n_arg), _basename(cpp_basename(_name)){}

File::File(DIR *d_arg, char *n_arg) :
  _file(NULL), _dirp(d_arg), _name(n_arg), _basename(cpp_basename(_name)){}

File::File(void) :_file(NULL), _dirp(NULL), _name(""), _basename(""){}

File::~File(void){}

size_t File::write(const uint8_t val){
  return fputc(val, _file);
}

size_t File::write(const uint8_t *buf, size_t size){
  return fwrite(buf, sizeof(char), size, _file);
}

int File::read(void){
  return fgetc(_file);
}

int File::peek(void){
  int c;
  c = fgetc(_file);
  ungetc(c, _file);
  return c;
}

int File::available(void){
  long current;
  long size;
  current = ftell(_file);
  fseek (_file, 0 ,SEEK_END);
  size = ftell(_file);
  fseek (_file, current, SEEK_SET);
  return (int)(size - current);
}

void File::flush(void){
  if (_file)
    fflush(_file);
}

int File::read(void *buf, uint16_t nbyte){
  return fread(buf, sizeof(char), nbyte, _file);
}

boolean File::seek(uint32_t pos){
  //error checking - position is valid?
  if(fseek(_file, pos, SEEK_SET) == 0){
    return true;
  } else {
    return false;
  }
}

uint32_t File::position(){
  return (uint32_t)ftell(_file);
}

// Should this be done with stat?
uint32_t File::size(){
  long current;
  long size;
  current = ftell(_file);
  fseek (_file, 0 ,SEEK_END);
  size = ftell(_file);
  fseek (_file, current, SEEK_SET);
  return (uint32_t)size;
}

void File::close(){
  if (!check_exists())
    return;

  if (_file != NULL)
    fclose(_file);

  _file = NULL;
  _dirp = NULL;
}

const char * File::name(void){
  return _basename.c_str();
}

boolean File::isDirectory(void){
  if (_file == NULL && _dirp != NULL)
    return true;

  if (_file != NULL && _dirp == NULL)
    return false;

  trace_error("isDirectory failed: File object not initialized\n");
  return false;
}

File File::openNextFile(uint8_t mode){

  struct dirent *dp;
  dp = readdir(_dirp);

  if(dp == NULL)
    return File();

  // skip these special directories
  // TODO: should we skip all hidden dot files/dirs?
  if (strcmp(dp->d_name,".") == 0)
    dp = readdir(_dirp);

  if (strcmp(dp->d_name,"..") == 0)
    dp = readdir(_dirp);

  // TODO make nextfile a std::string (concatenate with +)
  char nextfile[256];
  snprintf(nextfile, sizeof(nextfile),"%s/%s", _name.c_str(), dp->d_name);

  return SD.open(nextfile, mode);
}

void File::rewindDirectory(void){
  rewinddir(_dirp);
}

File::operator bool(){
  return NULL != _file || NULL != _dirp;
}

boolean File::check_exists(){
  if (!this) {
    trace_error("Empty or closed file in ");
    return false;
  }
  return true;
}

boolean SDClass::begin(uint8_t csPin){
  // cspin is ignored on x86 as it is not needed
  return check_sd();
}

File SDClass::open(const char *filepath, uint8_t mode){
  static const char R[] = "r";
  static const char W[] = "a+";
  const char *access_mode;
  switch(mode) {
    case FILE_READ:
      access_mode = R;
      break;
    case FILE_WRITE:
      access_mode = W;
      break;
    default:
      trace_error("File mode error\n");
      return File();
      break;
  }

  std::string abs_path = sd_path(filepath);

  // If the file doesnt exist return create a new file
  if(!SD.exists((char*)filepath)){
    FILE *f;
    f = fopen(abs_path.c_str(), access_mode);
    return File(f, (char*)filepath);
  }

  // Stat the file to see what it is
  struct stat st_buf;
  const int err = stat(abs_path.c_str(), &st_buf);
  if(err !=0) {
    trace_error("Failed to stat file %s, returned $d", abs_path.c_str(), err);
    return File(); // exceptions unfortunately not supported
  }
  // Check if it is a file or directory
  if (S_ISDIR(st_buf.st_mode)){
    DIR *dfd;
    dfd = opendir(abs_path.c_str());
    return File(dfd, (char*)filepath);
  } else if (S_ISREG (st_buf.st_mode)){
    FILE *f;
    f = fopen(abs_path.c_str(), access_mode);
    return File(f, (char*)filepath);
  }

  // If we get to here somethign went wrong...
  trace_error("not a file and not a directory %s", abs_path.c_str());
  return File();
}

boolean SDClass::exists(char *filepath){
  struct stat st;
  return stat(sd_path(filepath).c_str(), &st) == 0;
}

boolean SDClass::mkdir(char *filepath){
  char tmp[256];
  char *p = NULL;
  size_t len;

  snprintf(tmp, sizeof(tmp),"%s",filepath);
  len = strlen(tmp);
  if(tmp[len - 1] == '/')
    tmp[len - 1] = 0;
  for(p = tmp + 1; *p; p++)
    if(*p == '/') {
      *p = 0;
      ::mkdir(sd_path(tmp).c_str(), S_IRWXU);
      *p = '/';
    }
  return ::mkdir(sd_path(tmp).c_str(), S_IRWXU);
}

boolean SDClass::remove(char *filepath){
  return ::remove(sd_path(filepath).c_str());
}

boolean SDClass::rmdir(char *filepath){
  return ::rmdir(sd_path(filepath).c_str());
}

SDClass SD;
