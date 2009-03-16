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

 #ifndef KILOGRAM_H
 #define KILOGRAM_H

 // Libraries
 #include <stdexcept>

 // Namespace ReactPhysics3D
 namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Class Kilogram :
        The Kilogram class represents a mass unity. The value inside is a
        mass in kilogram.
    -------------------------------------------------------------------
*/
class Kilogram {
    private :
        double value;                           // Mass value in kilogram

    public :
        Kilogram();                                                 // Constructor
        Kilogram(double value) throw(std::invalid_argument);        // Constructor with arguments
        Kilogram(const Kilogram& mass);                             // Copy-constructor
        virtual ~Kilogram();                                        // Destructor

        double getValue() const;                                    // Return the mass value in kilogram
        void setValue(double value) throw(std::invalid_argument);   // Set the mass value
};

// --- Inlines functions --- //

// Return the value in kilogram
inline double Kilogram::getValue() const {
    return value;
}

// Set the value in kilogram
inline void Kilogram::setValue(double value) throw(std::invalid_argument) {

    // Check if the time value is positive
    if (value >= 0) {
        this->value = value;
    }
    else {
        // We throw an exception
        throw std::invalid_argument("Exception in Time: Wrong argument, a time value has to be positive");
    }
}

}

 #endif
