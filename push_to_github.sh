#!/bin/bash

echo "=========================================="
echo "GitHub'a Push İşlemi"
echo "=========================================="
echo ""
echo "Bu script tüm kodları GitHub'a push edecek."
echo ""
echo "Token'ınızı hazırladınız mı?"
echo "Token oluşturmak için: https://github.com/settings/tokens/new"
echo ""
read -p "Devam etmek için Enter'a basın..."

# Remote'u kontrol et
git remote -v

echo ""
echo "Push işlemi başlatılıyor..."
echo "İstendiğinde:"
echo "  Username: Atakanbas"
echo "  Password: Token'ınızı yapıştırın"
echo ""

# Master branch'i main'e push et
git push -u origin master:main

if [ $? -eq 0 ]; then
    echo ""
    echo "✅ Başarılı! Tüm kodlar GitHub'a push edildi!"
    echo "Repository: https://github.com/Atakanbas/o-mar"
else
    echo ""
    echo "❌ Push başarısız oldu. Token'ınızı kontrol edin."
fi

