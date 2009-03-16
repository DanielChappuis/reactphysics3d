/****************************************************************************
 * Copyright (C) 2009      Daniel Chappuis                                  *
 ****************************************************************************
 * This file is part of ReactPhysics3D.                                     *
 *                                                                          *
 * ReactPhysics3D is free software: you can redistribute it and/or modify   *
 * it under the terms of the GNU General Public License as published by     *
 * the Free Software Foundation, either version 3 of the License, or        *
 * (at your option) any later version.                                      *
 *                                                                          *
 * ReactPhysics3D is distributed in the hope that it will be useful,        *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of           *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the             *
 * GNU General Public License for more details.                             *
 *                                                                          *
 * You should have received a copy of the GNU General Public License        *
 * along with ReactPhysics3D. If not, see <http://www.gnu.org/licenses/>.   *
 ***************************************************************************/

 #ifndef TIME_H
 #define TIME_H

 // Libraries
 #include <stdexcept>
  #include <iostream>

 // Namespace ReactPhysics3D
 namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Class Time :
        The Time class represents a time unity. The value inside is the
        time value in seconds.
    -------------------------------------------------------------------
*/
class Time {
    private :
        double value;                   // Time in seconds

    public :
        Time();                                                         // Constructor
        Time(double value) throw(std::invalid_argument);                // Constructor with arguments
        Time(const Time& time);                                         // Copy-constructor
        virtual ~Time();                                                // Destructor

        double getValue() const;                                        // Return the time value in seconds
        void setValue(double value) throw(std::invalid_argument) ;      // Set the time value

        // Overloaded operators
        Time operator+(const Time& time2) const;                                // Overloaded operator for addition with Time
        Time operator-(const Time& time2) const throw(std::invalid_argument);   // Overloaded operator for substraction with Time
        Time operator*(double number) const throw(std::invalid_argument);       // Overloaded operator for multiplication with a number
};

// --- Inlines functions --- //

// Return the value in seconds
inline double Time::getValue() const {
    return value;
}

// Set the value in seconds
inline void Time::setValue(double value) throw(std::invalid_argument) {

    // Check if the time value is positive
    if (value >= 0.0) {
        this->value = value;
    }
    else {
        std::cout << "ERROR TIME : " << value << std::endl;

        // We throw an exception
        throw std::invalid_argument("Exception in test Time: Wrong argument, a time value has to be positive");
    }
}

// Overloaded operator for addition with Time
inline Time Time::operator+(const Time& time2) const {
    return Time(value + time2.value);
}

// Overloaded operator for substraction with Time
inline Time Time::operator-(const Time& time2) const throw(std::invalid_argument) {
    // Compute the result of the substraction
    double result = value - time2.value;

    // If the result is negative
    if (result <= 0.0) {
        // We throw an exception
        throw std::invalid_argument("Exception in Time::operator- : The result should be positive");
    }

    // Return the result
    return Time(result);
}

// Overloaded operator for multiplication with a number
inline Time Time::operator*(double number) const throw(std::invalid_argument) {
    // Check if the number is positive
    if (number > 0.0) {
        return Time(value*number);
    }
    else {
        // We throw an exception
        throw std::invalid_argument("Exception in Time::operator* : The argument number has to be positive");
    }
}

}

 #endif
