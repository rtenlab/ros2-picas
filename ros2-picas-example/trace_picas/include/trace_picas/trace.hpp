#ifndef TRACE_HPP_
#define TRACE_HPP_

#include <iostream>
#include <fstream>

namespace trace {

    class Trace {
        public:
            Trace(const std::string file_name);
            ~Trace();
            void trace_write(const std::string index, std::string value);
            void trace_write_count(const std::string index, std::string value, std::string count);
            
        private:
            const std::string file_name_;
            std::ofstream file_;

            
    };

}

#endif