/**
 * @file main.cpp
 * @brief Catch2 test runner main entry
 * 
 * This file provides the main entry point for Catch2 tests.
 * Tests are discovered automatically from test files.
 */

#include <catch2/catch_session.hpp>

int main(int argc, char* argv[])
{
    Catch::Session session;
    
    // Run tests
    int returnCode = session.applyCommandLine(argc, argv);
    if (returnCode != 0) {
        return returnCode;
    }
    
    return session.run();
}
