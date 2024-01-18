#define OUTPUT 0x20004

void hello()
{
        char *hello_string = "hello world!\n";
        while (*hello_string != 0)
                *((int *)OUTPUT) = *(hello_string++);
}
