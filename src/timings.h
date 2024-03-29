//                        0, 1, 2, 3, 4, 5, 6, 7, 8, 9, A, B, C, D, E ,F
const int cycleTable[] = {1, 1, 2, 2, 1, 1, 1, 2, 1, 1, 1, 1, 2, 2, 3, 3, // 0
                          3, 3, 3, 3, 1, 1, 1, 2, 1, 1, 1, 1, 3, 3, 3, 3, // 1
                          2, 2, 2, 2, 1, 1, 1, 2, 1, 1, 1, 1, 3, 3, 2, 2, // 2
                          2, 2, 2, 2, 1, 1, 1, 2, 1, 1, 1, 1, 4, 0, 0, 1, // 3
                          1, 1, 1, 3, 1, 1, 1, 2, 1, 1, 1, 1, 3, 3, 0, 0, // 4
                          1, 1, 1, 3, 1, 1, 1, 2, 1, 1, 1, 1, 4, 4, 4, 4, // 5
                          2, 2, 2, 3, 2, 2, 2, 1, 2, 2, 2, 2, 2, 2, 2, 2, // 6
                          1, 1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 4, 4, 4, 4, // 7
                          5, 3, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 4, 3, // 8
                          2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 4, 4, 1, 2, // 9
                          2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 3, 3, 4, 4, // A
                          2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, // B
                          1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 3, 3, 0, 0, // C
                          1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 6, 0, 0, 0, // D
                          2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, // E
                          1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 4, 4, 4, 4};// F

//                          0, 1, 2, 3, 4, 5, 6, 7, 8, 9, A, B, C, D, E ,F
const int CBCycleTable[] = {2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 ,2 ,2, // 0
                            2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, // 1
                            2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, // 2
                            2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, // 3
                            2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, // 4
                            2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, // 5
                            4, 4, 4, 4, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, // 6
                            2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, // 7
                            2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, // 8
                            2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, // 9
                            2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, // A
                            2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, // D
                            2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, // B
                            2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, // D
                            4, 4, 4, 4, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, // E
                            2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2};// F