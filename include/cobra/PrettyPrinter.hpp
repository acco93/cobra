/**
 * Printer utility.
 */

#ifndef COBRA_PRETTYPRINTER_HPP
#define COBRA_PRETTYPRINTER_HPP

#include <utility>
#include <vector>
#include <iostream>
#include <iomanip>

namespace cobra {

class PrettyPrinter {

 public:

    class Field {

     public:

        enum Type { INTEGER, REAL, STRING };

        Field(std::string name_, Type type_ = Type::REAL, int max_width_ = 8, std::string sep_ = "\t") :
             name(std::move(name_)), type(type_), max_width(max_width_), sep(std::move(sep_)) { };

        inline const std::string &get_name() const { return name; }
        inline const Type &get_type() const { return type; }
        inline const int &get_max_width() const { return max_width; }
        inline const std::string &get_separator() const { return sep; }

     private:
        std::string name;
        Type type;
        int max_width;
        std::string sep;
    };

    explicit PrettyPrinter(std::vector<Field>  args_) : args(std::move(args_)) {}

    template<typename T, typename... Values>
    void print(T t, Values... values) { // recursive variadic function

        if (!header_count) {

            header_count = max_header_count;

            std::cout << "\n";

            for (const auto& header : args) {
                for (auto n = 0; n < header.get_max_width() + 2; n++) {
                    std::cout << "*";
                }
                std::cout << header.get_separator();
            }
            std::cout << "\n";

            for (const auto& header : args) {
                std::cout << " ";
                std::cout << std::fixed;
                std::cout << std::setprecision(real_precision);
                std::cout << std::setw(header.get_max_width());
                std::cout << header.get_name();
                std::cout << std::setprecision(10);
                std::cout << std::defaultfloat;
                std::cout << " ";
                std::cout << header.get_separator();
            }

            std::cout << "\n";

            for (const auto& header : args) {
                for (auto n = 0; n < header.get_max_width() + 2; n++) {
                    std::cout << "*";
                }
                std::cout << header.get_separator();
            }

            std::cout << "\n";

            for (const auto& header : args) {
                for (auto n = 0; n < header.get_max_width() + 2; n++) {
                    std::cout << " ";
                }
                std::cout << header.get_separator();
            }
            std::cout << "\n";

        }

        header_count--;

        print_impl(0, t, values...);

    }

    void notify(const std::string &message) {
        std::cout << "\n";
        std::cout << message << "\n";
        std::cout << "\n";
    }

 private:

    template<typename T>
    void just_print(T value, Field &header) {
        std::cout << " ";
        std::cout << std::fixed;
        std::cout << std::setprecision(real_precision);
        std::cout << std::setw(header.get_max_width());
        switch (header.get_type()) {
            case Field::INTEGER:std::cout << static_cast<int>(value);
                break;
            case Field::REAL:std::cout << static_cast<float>(value);
                break;
            case Field::STRING:std::cout << value;
                break;
        }
        std::cout << " ";
        std::cout << std::setprecision(10);
        std::cout << std::defaultfloat;
    }

    template<typename T>
    void print_impl(int n, T value) {

        if (static_cast<unsigned>(n)==args.size()) {
            std::cout << "Values do not correspond to headers\n";
            return;
        }

        just_print(value, args[n]);
        std::cout << "\n";

    }

    template<typename T, typename... Values>
    void print_impl(int n, T value, Values... values) {   // recursive variadic function

        if (static_cast<unsigned>(n)==args.size()) {
            std::cout << "Values do not correspond to headers\n";
            return;
        }

        just_print(value, args[n]);
        std::cout << args[n].get_separator();

        print_impl(n + 1, values...);

    }

    std::vector<Field> args;
    const int max_header_count = 15;
    int header_count = 0;
    const int real_precision = 2;

};

}

#endif //COBRA_PRETTYPRINTER_HPP
