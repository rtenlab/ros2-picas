#include "trace_picas/trace.hpp"

namespace trace {
    
    Trace::Trace(const std::string fname) : file_name_(fname) {
        file_.open(file_name_);
    }

    Trace::~Trace() {
        file_.close();
    }

    void Trace::trace_write(const std::string index, std::string value) {
            file_ << index << " " << value << std::endl;
    }

    void Trace::trace_write_count(const std::string index, std::string value, std::string count) {
            file_ << index << " " << value << " " << count << std::endl;
    }

}
