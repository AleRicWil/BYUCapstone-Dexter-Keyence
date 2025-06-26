#------------------------------------------------------------
# Makefile for LJS communication library
#------------------------------------------------------------
LIB_TARGET	= libljscom

# Directories
INCDIR		= ../include
LIBSRC_DIR	= ../libsrc
OBJDIR		= obj

.PHONY:	solib
solib: directories $(LIB_TARGET).so

directories:
	@mkdir -p -v $(OBJDIR)

.PHONY: clean
clean:
	rm -rf $(OBJDIR) $(LIB_TARGET).so

#------------------------------------------------------------
# Compile/Archive library
#------------------------------------------------------------
CC		= g++
LIB_CFLAGS	= -fPIC -g -Wall -pthread -I$(INCDIR)
LIB_SUB1	= LJS8_IF_Linux
LIB_SUB2	= LJS_ACQ

LIB_OBJS	= $(OBJDIR)/$(LIB_SUB1).o $(OBJDIR)/$(LIB_SUB2).o

$(LIB_TARGET).so: $(LIB_OBJS)
	$(CC) -fPIC -shared -o $@ $(LIB_OBJS)

$(OBJDIR)/$(LIB_SUB1).o: $(LIBSRC_DIR)/$(LIB_SUB1).cpp
	$(CC) $(LIB_CFLAGS) -c $(LIBSRC_DIR)/$(LIB_SUB1).cpp -o $@

$(OBJDIR)/$(LIB_SUB2).o: $(LIBSRC_DIR)/$(LIB_SUB2).cpp
	$(CC) $(LIB_CFLAGS) -c $(LIBSRC_DIR)/$(LIB_SUB2).cpp -o $@

