/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#warning "foo was here"

class sub
{
public:
    int num=123123;
};
class foobar : public sub
{
public:
    int x=555;
    foobar() : sub()
    {
        
    }
    int giveme(void)
    {
        return -456;
    }
};
