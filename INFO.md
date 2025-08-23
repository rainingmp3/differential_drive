2. What formats you can use

They are the same as in printf:

%s → C string (const char*)
(if you have std::string, call .c_str())
%d → int
%u → unsigned int
%f → double / float
%ld, %lu → long / unsigned long
%p → pointer
%zu → size_t
