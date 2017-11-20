//
// Created by chke on 11/20/17.
//

#ifndef PROJECT_DEBUG_H
#define PROJECT_DEBUG_H

#define __DEBUG__

#ifdef __DEBUG__
//#define DEBUG(format,...) printf("file"__FILE__ format,##__VA_ARGS__)
#define DEBUG(format,...) printf(format, ##__VA_ARGS__)
#else
#define DEBUG(format,...)
#endif

#endif //PROJECT_DEBUG_H
