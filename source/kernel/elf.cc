/*
 *
 * Copyright (c) 2016 Raphine Project
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * Author: hikalium, Liva
 * 
 */

#include <tty.h>
#include <elf.h>
#include <string.h>
#include <stdlib.h>

void ElfObject::Init() {
  // IA32_EFER.SCE = 1
  //
  if(!IsElf() || !IsElf64() || (!IsOsabiSysv() && !IsOsabiGnu())){
    gtty->Printf("Not supported module type.\n");
    return;
  }
  gtty->Printf("ABI: %d\n", _ehdr->e_ident[EI_OSABI]);
  gtty->Printf("Entry point is 0x%08x \n", _ehdr->e_entry);

  const Elf64_Shdr *shstr = &reinterpret_cast<const Elf64_Shdr *>(_head + _ehdr->e_shoff)[_ehdr->e_shstrndx];
  
  Elf64_Xword total_memsize = 0;

  gtty->Printf("Sections:\n");
  if (_ehdr->e_shstrndx != SHN_UNDEF) {
    const char *strtab = reinterpret_cast<const char *>(_head + shstr->sh_offset);
    for(int i = 0; i < _ehdr->e_shnum; i++){
      const Elf64_Shdr *shdr = (const Elf64_Shdr *)(_head + _ehdr->e_shoff + _ehdr->e_shentsize * i);
      const char *sname = strtab + shdr->sh_name;
      gtty->Printf(" [%2d] %s size: %x offset: %x\n", i, sname, shdr->sh_size, shdr->sh_offset);
      if (total_memsize < shdr->sh_addr + shdr->sh_offset) {
        total_memsize = shdr->sh_addr + shdr->sh_offset;
      }
    }
  }

  uint8_t *membuffer;
  bool page_mapping = false;
  if (_ehdr->e_type == ET_DYN || _ehdr->e_type == ET_REL) {
    membuffer = reinterpret_cast<uint8_t *>(malloc(total_memsize));
  } else if (_ehdr->e_type == ET_EXEC) {
    membuffer = reinterpret_cast<uint8_t *>(0);
    page_mapping = true;
  } else {
    kassert(false);
  }
  gtty->Printf("membuffer: 0x%llx (%llx)\n", membuffer, total_memsize);
  // PT_LOADとなっているセグメントをメモリ上にロード
  for(int i = 0; i < _ehdr->e_phnum; i++){
    const Elf64_Phdr *phdr = (const Elf64_Phdr *)(_head + _ehdr->e_phoff + _ehdr->e_phentsize * i);

    if (page_mapping && phdr->p_memsz != 0) {
      virt_addr start = ptr2virtaddr(membuffer) + phdr->p_vaddr;
      virt_addr end = start + phdr->p_memsz;
      _loader.MapAddr(start, end);
    }
    
    switch(phdr->p_type){
      case PT_LOAD:
        gtty->Printf("phdr[%d]: Load to +0x%llx\n", i, phdr->p_vaddr);
        memcpy(membuffer + phdr->p_vaddr, &_head[phdr->p_offset], phdr->p_filesz);
        break;
    }
  }
  // セクション .bss を0クリア
  for(int i = 0; i < _ehdr->e_shnum; i++){
    const Elf64_Shdr *shdr = (const Elf64_Shdr *)(_head + _ehdr->e_shoff + _ehdr->e_shentsize * i);
    if (shdr->sh_type == SHT_NOBITS) {
      if ((shdr->sh_flags & SHF_ALLOC) != 0) {
        memset(membuffer + shdr->sh_addr, 0, shdr->sh_size);
      }
    }
  }

  _entry = reinterpret_cast<FType>(membuffer + _ehdr->e_entry);

  free(membuffer);
}
