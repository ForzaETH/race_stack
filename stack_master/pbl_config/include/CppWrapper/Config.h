#ifndef CONFIG_H
#define CONFIG_H
#include <string>
 // Everything needed for embedding
#include <iostream>

class Config
{
public:
    Config() {

    };
    ~Config() {};
    virtual void load_config(std::string racecar_version) = 0;
};
#endif  // CONFIG_H