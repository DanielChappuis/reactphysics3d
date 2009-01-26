/****************************************************************************
 * Copyright (C) 2008      Daniel Chappuis                                  *
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

// TODO exceptions.h : Check if all expressions are usefull and are correct (in Matrix3x3.h methods throw std::invalid_argument MatrixException doesn't inherit from
//                     std::invalid_argument
#ifndef EXCEPTIONS_H
#define EXCEPTIONS_H

// Libraries
#include <stdexcept>

/*  -------------------------------------------------------------------
    Exception class for the mathematics library
    -------------------------------------------------------------------
*/

// MathException
class MathException : public std::runtime_error {
    public:
        MathException(const std::string& msg="MathException");                  // Constructor
        virtual ~MathException() throw();                                       // Destructor
        virtual const char* what() const throw();                               // Overriding the base exception method
};


// DivisionByZeroException
class DivisionByZeroException : public MathException {
    public:
        DivisionByZeroException(const std::string& msg="DivisionByZeroException : Division by zero !");     // Constructor
        virtual ~DivisionByZeroException() throw();                                                         // Destructor
        virtual const char* what() const throw();                                                           // Overriding the base exception method
};

// Matrix Exception class
class MatrixException : public MathException {
    public:
        MatrixException(const std::string& msg="MatrixException");              // Constructor
        virtual ~MatrixException() throw();                                     // Destructor
        virtual const char* what() const throw();                               // Overriden exception base class method
};

// VectorException class
class VectorException : public std::runtime_error {
    public :
        VectorException(const std::string& msg="VectorException");                  // Constructor of the VectorException class
        virtual ~VectorException() throw();                                         // Destructor of the VectorException class
        virtual const char* what() const throw();                                   // Overriding the base exception method
};

// QuaternionException class
class QuaternionException : public std::runtime_error {
    public :
        QuaternionException(const std::string& msg="QuaternionException");           // Constructor of the QuaternionException class
        virtual ~QuaternionException() throw();                                      // Destructor of the QuaternionException class
        virtual const char* what() const throw();                                    // Overriding the base exception method
};


#endif
