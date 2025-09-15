#!/usr/bin/env python3
"""
Lister les ports COM disponibles et proposer une sélection dans le terminal.
Dépendance : pyserial (`pip install pyserial`)
"""

from typing import Optional
import sys
from serial.tools import list_ports


def list_serial_ports():
    """Renvoie la liste d'objets list_ports_common.ListPortInfo"""
    return list(list_ports.comports())


def pretty_print_ports(ports):
    if not ports:
        print("Aucun port série détecté.")
        return
    print("Ports série détectés :")
    for i, p in enumerate(ports, start=1):
        # p.device : nom du port (ex: COM3 ou /dev/ttyUSB0)
        # p.description, p.hwid : info supplémentaires
        print(f"  [{i}] {p.device}\t- {p.description}\t({p.hwid})")


def ask_user_choice(ports) -> Optional[str]:
    """
    Demande à l'utilisateur de choisir un port et renvoie p.device (str),
    ou None si annulation / pas de choix.
    """
    if not ports:
        return None

    if len(ports) == 1:
        # Si un seul port, proposer de l'utiliser directement
        only = ports[0].device
        ans = input(f"Un seul port trouvé ({only}). L'utiliser ? [Y/n] ").strip().lower()
        if ans in ("", "y", "yes", "o", "oui"):
            return only
        return None

    while True:
        s = input("Sélectionnez un port en tapant son numéro (ou 'q' pour quitter) : ").strip()
        if s.lower() in ("q", "quit", "exit"):
            return None
        if not s:
            print("Entrez un numéro ou 'q'.")
            continue
        if not s.isdigit():
            print("Entrée invalide : tapez un numéro.")
            continue
        n = int(s)
        if 1 <= n <= len(ports):
            return ports[n - 1].device
        print(f"Numéro hors plage (1..{len(ports)}).")


def select_com_port() -> Optional[str]:
    ports = list_serial_ports()
    pretty_print_ports(ports)
    chosen = ask_user_choice(ports)
    if chosen:
        print(f"Port sélectionné : {chosen}")
    else:
        print("Aucun port sélectionné.")
    return chosen


if __name__ == "__main__":
    select_com_port()